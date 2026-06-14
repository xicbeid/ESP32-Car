/*
 * ESP32-P4 UART Motor + WiFi AP + Web Control
 *
 * UART0 GPIO37/38 -> Motor driver board (text protocol)
 * WiFi AP via ESP-Hosted (C6 SDIO)
 * HTTP server port 80: web remote control
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "uart_module.h"
#include "app_motor_uart.h"
#include "wifi_module.h"
#include "web_control.h"
#include "camera_module.h"

#define delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))


#define MOTOR_TYPE 2     /* 1:520 2:310 3:TT-encoder 4:TT-DC 5:L-520 */
#define UPLOAD_DATA 1    /* 0:none 1:total encoder 2:realtime encoder 3:speed mm/s */

#define WIFI_AP_SSID     "ESP32-Car"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_CHANNEL  1

static const char *TAG = "MAIN";

/* ==================== Motor Command Queue ==================== */
typedef struct {
    motor_cmd_t cmd;
    int         distance_cm;
    int         speed;
} motor_msg_t;

static QueueHandle_t g_motor_queue = NULL;
static volatile bool g_motor_stop_flag = false;

/* ==================== Encoder Closed-Loop Constants ==================== */
#if MOTOR_TYPE == 1   /* 520 motor */
  #define PULSE_PHASE   30
  #define PULSE_LINE    11
  #define WHEEL_DIA_MM  67.00f
#elif MOTOR_TYPE == 2 /* 310 motor */
  #define PULSE_PHASE   20
  #define PULSE_LINE    13
  #define WHEEL_DIA_MM  48.00f
#elif MOTOR_TYPE == 3 /* TT encoder */
  #define PULSE_PHASE   45
  #define PULSE_LINE    13
  #define WHEEL_DIA_MM  68.00f
#elif MOTOR_TYPE == 4 /* TT-DC (no encoder) */
  #define PULSE_PHASE   48
  #define PULSE_LINE    1
  #define WHEEL_DIA_MM  65.00f
#elif MOTOR_TYPE == 5 /* L-520 */
  #define PULSE_PHASE   40
  #define PULSE_LINE    11
  #define WHEEL_DIA_MM  67.00f
#endif

#define PULSES_PER_REV    (PULSE_PHASE * PULSE_LINE)
#define WHEEL_CIRCUM_MM   (3.14159265f * WHEEL_DIA_MM)   /* mm per revolution */
#define PULSES_PER_MM     ((float)PULSES_PER_REV / WHEEL_CIRCUM_MM)
#define PULSES_PER_CM     (PULSES_PER_MM * 10.0f)

/* max time to wait for encoder, per cm (ms) — generous fallback */
#define ENC_TIMEOUT_PER_CM  500

static int32_t encoder_base[4] = {0};   /* snapshot before movement starts */
static int32_t enc_last_log[4] = {0};   /* last logged delta for debug */

/* Convert cm to encoder pulses (rounded) */
static inline int32_t cm_to_pulses(int cm)
{
    return (int32_t)(cm * PULSES_PER_CM + 0.5f);
}

/* Consume pending encoder data, update Encoder_Now[0..3] */
static inline void encoder_refresh(void)
{
    if (g_recv_flag) {
        g_recv_flag = 0;
        Deal_data_real();
    }
}

/* Wait until all 4 motors reach their target encoder deltas.
 * Returns true on success, false on stop-flag or timeout. */
static bool motor_encoder_wait(const int32_t target[4], int timeout_ms)
{
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        encoder_refresh();

        if (g_motor_stop_flag) {
            Contrl_Speed(0, 0, 0, 0);
            ESP_LOGI(TAG, "Enc: stopped by flag at %dms", elapsed);
            return false;
        }

        bool done = true;
        for (int i = 0; i < 4; i++) {
            if (Encoder_Now[i] - encoder_base[i] < target[i]) { done = false; break; }
        }
        if (done) {
            Contrl_Speed(0, 0, 0, 0);
            for (int i = 0; i < 4; i++) enc_last_log[i] = Encoder_Now[i] - encoder_base[i];
            ESP_LOGI(TAG, "Enc: done @%dms d=[%ld %ld %ld %ld]",
                     elapsed, (long)enc_last_log[0], (long)enc_last_log[1],
                     (long)enc_last_log[2], (long)enc_last_log[3]);
            return true;
        }
        delay_ms(10);
        elapsed += 10;
    }
    /* Timeout — stop and warn */
    Contrl_Speed(0, 0, 0, 0);
    encoder_refresh();
    ESP_LOGW(TAG, "Enc: timeout %dms d=[%ld %ld %ld %ld]",
             elapsed, (long)(Encoder_Now[0] - encoder_base[0]),
             (long)(Encoder_Now[1] - encoder_base[1]),
             (long)(Encoder_Now[2] - encoder_base[2]),
             (long)(Encoder_Now[3] - encoder_base[3]));
    return false;
}

/* Snapshot current encoder as zero. Waits up to 100ms for a fresh reading. */
static void encoder_snapshot(void)
{
    int w = 0;
    while (!g_recv_flag && w < 100) { delay_ms(10); w += 10; }
    encoder_refresh();
    for (int i = 0; i < 4; i++) encoder_base[i] = Encoder_Now[i];
}

/* ==================== Motor Task (encoder closed-loop) ==================== */
static void motor_task(void *arg)
{
    motor_msg_t msg;

    while (1) {
        if (xQueueReceive(g_motor_queue, &msg, portMAX_DELAY) != pdTRUE)
            continue;

        int ms = msg.speed * 10;
        if (ms < 100) ms = 100;
        if (ms > 1000) ms = 1000;

        int32_t dist_pulses = cm_to_pulses(msg.distance_cm);
        int32_t t[4] = {0};
        int timeout = msg.distance_cm * ENC_TIMEOUT_PER_CM + 5000;

        ESP_LOGI(TAG, "Motor: cmd=%d dist=%dcm(%ldp) speed=%d timeout=%dms",
                 msg.cmd, msg.distance_cm, (long)dist_pulses, msg.speed, timeout);

        switch (msg.cmd) {
        case MOTOR_CMD_STOP:
            break;

        case MOTOR_CMD_FORWARD:
        case MOTOR_CMD_BACKWARD: {
            g_motor_stop_flag = false;
            encoder_snapshot();
            t[0] = t[1] = t[2] = t[3] = dist_pulses;
            int16_t s = (msg.cmd == MOTOR_CMD_FORWARD) ? ms : -ms;
            Contrl_Speed(s, s, s, s);
            motor_encoder_wait(t, timeout);
            break;
        }
        case MOTOR_CMD_LEFT:
            g_motor_stop_flag = false;
            encoder_snapshot();
            t[0] = t[1] = dist_pulses;
            t[2] = t[3] = dist_pulses;
            Contrl_Speed(-ms, -ms, ms, ms);
            motor_encoder_wait(t, timeout);
            break;

        case MOTOR_CMD_RIGHT:
            g_motor_stop_flag = false;
            encoder_snapshot();
            t[0] = t[1] = dist_pulses;
            t[2] = t[3] = dist_pulses;
            Contrl_Speed(ms, ms, -ms, -ms);
            motor_encoder_wait(t, timeout);
            break;
        }
    }
}

/* ==================== Motor Control Callback (from HTTP) ==================== */
static void motor_control_callback(motor_cmd_t cmd, int distance_cm, int speed)
{
    if (cmd == MOTOR_CMD_STOP) {
        /* Immediate stop via flag + UART */
        g_motor_stop_flag = true;
        Contrl_Speed(0, 0, 0, 0);
        motor_msg_t junk;
        while (xQueueReceive(g_motor_queue, &junk, 0) == pdTRUE) { }
        ESP_LOGI(TAG, "Motor: STOP (immediate)");
        return;
    }

    motor_msg_t msg = { .cmd = cmd, .distance_cm = distance_cm, .speed = speed };
    if (xQueueSend(g_motor_queue, &msg, 0) != pdTRUE) {
        Contrl_Speed(0, 0, 0, 0);
        ESP_LOGW(TAG, "Motor queue full, forced stop");
    }
}

/* ==================== Motor Init ==================== */
static void motor_init(void)
{
    ESP_LOGI(TAG, "Init UART0 (TX=%d RX=%d 115200)...", UART0_TX_PIN, UART0_RX_PIN);
    uart0_init();
    delay_ms(100);

    printf("please wait...\r\n");

    /* Disable upload first */
    send_upload_data(false, false, false);
    delay_ms(10);

    /* Configure motor parameters */
#if MOTOR_TYPE == 1
    send_motor_type(1);  delay_ms(100);
    send_pulse_phase(30); delay_ms(100);
    send_pulse_line(11);  delay_ms(100);
    send_wheel_diameter(67.00); delay_ms(100);
    send_motor_deadzone(1600);  delay_ms(100);
#elif MOTOR_TYPE == 2
    send_motor_type(2);  delay_ms(100);
    send_pulse_phase(20); delay_ms(100);
    send_pulse_line(13);  delay_ms(100);
    send_wheel_diameter(48.00); delay_ms(100);
    send_motor_deadzone(1300);  delay_ms(100);
#elif MOTOR_TYPE == 3
    send_motor_type(3);  delay_ms(100);
    send_pulse_phase(45); delay_ms(100);
    send_pulse_line(13);  delay_ms(100);
    send_wheel_diameter(68.00); delay_ms(100);
    send_motor_deadzone(1250);  delay_ms(100);
#elif MOTOR_TYPE == 4
    send_motor_type(4);  delay_ms(100);
    send_pulse_phase(48); delay_ms(100);
    send_motor_deadzone(1000);  delay_ms(100);
#elif MOTOR_TYPE == 5
    send_motor_type(1);  delay_ms(100);
    send_pulse_phase(40); delay_ms(100);
    send_pulse_line(11);  delay_ms(100);
    send_wheel_diameter(67.00); delay_ms(100);
    send_motor_deadzone(1600);  delay_ms(100);
#endif

    /* Enable encoder upload */
#if UPLOAD_DATA == 1
    send_upload_data(true, false, false);
#elif UPLOAD_DATA == 2
    send_upload_data(false, true, false);
#elif UPLOAD_DATA == 3
    send_upload_data(false, false, true);
#endif
    delay_ms(10);

    /* Start UART RX task */
    xTaskCreate(UART_Process_Task, "UART_RX", 4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "Motor type %d ready (UART0 TX=%d RX=%d)", MOTOR_TYPE, UART0_TX_PIN, UART0_RX_PIN);
}

/* ==================== Encoder Monitor Task ==================== */
static void encoder_monitor_task(void *arg)
{
    while (1) {
        encoder_refresh();
        delay_ms(20);
    }
}

/* ==================== Main ==================== */
void app_main(void)
{
    esp_err_t ret;

    /* Suppress httpd socket noise (ECONNRESET=104 from mobile polling/disconnect) */
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
    esp_log_level_set("httpd", ESP_LOG_WARN);
    /* Suppress harmless sensor auto-detect failures (OV5647 etc.) — NONE silences all */
    esp_log_level_set("ov5647", ESP_LOG_NONE);
    esp_log_level_set("sccb_i2c", ESP_LOG_ERROR);
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);
    /* SDIO idle-state events are harmless ESP-Hosted transport noise */
    esp_log_level_set("sdmmc_req", ESP_LOG_NONE);

    /* 1. NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    /* 2. CSI Camera — init early to provide XCLK (official example pattern) */
    ESP_LOGI(TAG, "Initializing CSI camera...");
    esp_err_t cam_err = camera_module_init(NULL);
    if (cam_err != ESP_OK) {
        ESP_LOGW(TAG, "Camera init failed (%s) — running without camera", esp_err_to_name(cam_err));
    }

    /* 3. Motor init — UART0 re-init (tears down console UART, installs with RX queue) */
    motor_init();

    /* 4. Encoder monitor task */
    xTaskCreate(encoder_monitor_task, "Enc_Mon", 2048, NULL, 1, NULL);

    /* 5. Motor command queue + task — must exist before web_control_start */
    g_motor_queue = xQueueCreate(3, sizeof(motor_msg_t));
    if (!g_motor_queue) { ESP_LOGE(TAG, "Motor queue create fail"); }
    xTaskCreate(motor_task, "Motor_Task", 4096, NULL, 5, NULL);

    /* 6. WiFi AP */
    ESP_ERROR_CHECK(wifi_module_init_netstack());
    ESP_ERROR_CHECK(wifi_module_init_ap(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL));

    /* 7. Web control (with motor callback) */
    ESP_ERROR_CHECK(web_control_start(motor_control_callback));

    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, " WiFi AP  : %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, " Password : %s", WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, " Control  : http://192.168.4.1/");
    ESP_LOGI(TAG, " Camera   : http://192.168.4.1/stream");
    ESP_LOGI(TAG, " Motor    : UART0 (TX=%d RX=%d) type=%d", UART0_TX_PIN, UART0_RX_PIN, MOTOR_TYPE);
    ESP_LOGI(TAG, "================================================");

    while (1) { delay_ms(1000); }
}

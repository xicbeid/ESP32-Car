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

/* Time-based distance (no encoder needed) */
#define MS_PER_CM_FWD   8    /* ms per cm forward */
#define MS_PER_CM_TURN  12   /* ms per cm turn (slower due to differential) */

static void motor_task(void *arg)
{
    motor_msg_t msg;

    while (1) {
        if (xQueueReceive(g_motor_queue, &msg, portMAX_DELAY) != pdTRUE)
            continue;

        int ms = msg.speed * 10;
        if (ms < 100) ms = 100;
        if (ms > 1000) ms = 1000;

        ESP_LOGI(TAG, "Motor: cmd=%d dist=%d speed=%d", msg.cmd, msg.distance_cm, msg.speed);

        switch (msg.cmd) {
        case MOTOR_CMD_STOP:
            g_motor_stop_flag = true;
            Contrl_Speed(0, 0, 0, 0);
            g_motor_stop_flag = false;
            break;
        case MOTOR_CMD_FORWARD:
            g_motor_stop_flag = false;
            Contrl_Speed(ms, ms, ms, ms);
            delay_ms(msg.distance_cm * MS_PER_CM_FWD);
            if (!g_motor_stop_flag) Contrl_Speed(0, 0, 0, 0);
            break;
        case MOTOR_CMD_BACKWARD:
            g_motor_stop_flag = false;
            Contrl_Speed(-ms, -ms, -ms, -ms);
            delay_ms(msg.distance_cm * MS_PER_CM_FWD);
            if (!g_motor_stop_flag) Contrl_Speed(0, 0, 0, 0);
            break;
        case MOTOR_CMD_LEFT:
            g_motor_stop_flag = false;
            Contrl_Speed(-ms, -ms, ms, ms);
            delay_ms(msg.distance_cm * MS_PER_CM_TURN);
            if (!g_motor_stop_flag) Contrl_Speed(0, 0, 0, 0);
            break;
        case MOTOR_CMD_RIGHT:
            g_motor_stop_flag = false;
            Contrl_Speed(ms, ms, -ms, -ms);
            delay_ms(msg.distance_cm * MS_PER_CM_TURN);
            if (!g_motor_stop_flag) Contrl_Speed(0, 0, 0, 0);
            break;
        }
    }
}

/* ==================== Motor Control Callback (from HTTP) ==================== */
static void motor_control_callback(motor_cmd_t cmd, int distance_cm, int speed)
{
    motor_msg_t msg = { .cmd = cmd, .distance_cm = distance_cm, .speed = speed };
    if (cmd == MOTOR_CMD_STOP) {
        g_motor_stop_flag = true;
    }
    if (xQueueSend(g_motor_queue, &msg, 0) != pdTRUE) {
        Contrl_Speed(0, 0, 0, 0);
        ESP_LOGW(TAG, "Motor queue full, forced stop");
    }
}

/* ==================== Motor Init (preserve original UART0 config) ==================== */
static void motor_init(void)
{
    ESP_LOGI(TAG, "Init UART0 (TX=37 RX=38 115200)...");
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

    ESP_LOGI(TAG, "Motor type %d ready (UART0 TX=37 RX=38)", MOTOR_TYPE);
}

/* ==================== Encoder Monitor Task ==================== */
static void encoder_monitor_task(void *arg)
{
    while (1) {
        if (g_recv_flag) {
            g_recv_flag = 0;
            Deal_data_real();
        }
        delay_ms(100);
    }
}

/* ==================== Main ==================== */
void app_main(void)
{
    esp_err_t ret;

    /* 1. NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    /* 2. Motor init (UART0 + config) */
    motor_init();

    /* 3. Encoder monitor task */
    xTaskCreate(encoder_monitor_task, "Enc_Mon", 2048, NULL, 1, NULL);

    /* 4. Motor command queue + task */
    g_motor_queue = xQueueCreate(3, sizeof(motor_msg_t));
    if (!g_motor_queue) { ESP_LOGE(TAG, "Queue fail"); return; }
    xTaskCreate(motor_task, "Motor_Task", 4096, NULL, 5, NULL);

    /* 5. WiFi AP */
    ESP_ERROR_CHECK(wifi_module_init_netstack());
    ESP_ERROR_CHECK(wifi_module_init_ap(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL));

    /* 6. Web control */
    ESP_ERROR_CHECK(web_control_start(motor_control_callback));

    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, " WiFi AP  : %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, " Password : %s", WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, " Control  : http://192.168.4.1/");
    ESP_LOGI(TAG, " Motor    : UART0 (TX=37 RX=38) type=%d", MOTOR_TYPE);
    ESP_LOGI(TAG, "================================================");

    while (1) { delay_ms(1000); }
}

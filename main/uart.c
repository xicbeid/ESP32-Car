/*
 * ESP32-P4 UART 电机 + WiFi AP + 网页控制
 *
 * UART0 GPIO20/21 → 电机驱动板 (文本协议)
 * WiFi AP 通过 ESP-Hosted (C6 SDIO)
 * HTTP 服务器 :80 端口: 网页遥控
 *
 * 双模式操控:
 *   速度模式 (D-Pad)  — 按住移动松手即停 (直接调速，不用编码器)
 *   位置模式 (GO按钮) — 编码器闭环走到目标距离，按预设速度
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
#include "imu_usb.h"

#define delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))


#define MOTOR_TYPE 2     /* 1:520 2:310 3:TT-encoder 4:TT-DC 5:L-520 */
#define UPLOAD_DATA 1    /* 0:不上传 1:累计编码器 2:实时编码器 3:速度 mm/s */

#define WIFI_AP_SSID     "ESP32-Car"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_CHANNEL  1

static const char *TAG = "MAIN";

/* ── 电机命令队列 ── */
typedef struct {
    motor_cmd_t cmd;
    int         distance_cm;
    int         speed;
} motor_msg_t;

static QueueHandle_t g_motor_queue = NULL;
volatile bool g_motor_stop_flag = false;
volatile bool g_velocity_active = false;  /* D-Pad 速度模式激活时置 true */
volatile bool g_position_active = false;  /* GO 编码器移动执行中置 true */

/* ── 编码器闭环常量 ── */
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
#define WHEEL_CIRCUM_MM   (3.14159265f * WHEEL_DIA_MM)   /* 每转周长 mm */
#define PULSES_PER_MM     ((float)PULSES_PER_REV / WHEEL_CIRCUM_MM)
#define PULSES_PER_CM     (PULSES_PER_MM * 10.0f)

/* 编码器超时等待，每 cm 最大等待 (ms) — 宽松的兜底值 */
#define ENC_TIMEOUT_PER_CM  500

static int32_t encoder_base[4] = {0};   /* 移动前的编码器快照 */
static int32_t enc_last_log[4] = {0};   /* 最近记录的增量 (调试用) */

/* cm 转编码器脉冲 (四舍五入) */
static inline int32_t cm_to_pulses(int cm)
{
    return (int32_t)(cm * PULSES_PER_CM + 0.5f);
}

/* 消费待处理的编码器数据，更新 Encoder_Now[0..3] */
static inline void encoder_refresh(void)
{
    if (g_recv_flag) {
        g_recv_flag = 0;
        Deal_data_real();
    }
}

/* 等待 4 个电机编码器增量均达到目标值。
 * 成功返回 true, stop_flag/超时返回 false。
 * g_velocity_active 为 true 时提前返回但不发送零速命令
 * (速度模式持有电机控制权)。 */
static bool motor_encoder_wait(const int32_t target[4], int timeout_ms)
{
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        encoder_refresh();

        if (g_motor_stop_flag) {
            if (!g_velocity_active) {
                Contrl_Speed(0, 0, 0, 0);
                ESP_LOGI(TAG, "编码器: 停止标志触发 @%dms", elapsed);
            } else {
                ESP_LOGI(TAG, "编码器: 速度模式取消 @%dms", elapsed);
            }
            return false;
        }

        bool done = true;
        for (int i = 0; i < 4; i++) {
            if (Encoder_Now[i] - encoder_base[i] < target[i]) { done = false; break; }
        }
        if (done) {
            Contrl_Speed(0, 0, 0, 0);
            for (int i = 0; i < 4; i++) enc_last_log[i] = Encoder_Now[i] - encoder_base[i];
            ESP_LOGI(TAG, "编码器: 完成 @%dms 增量=[%ld %ld %ld %ld]",
                     elapsed, (long)enc_last_log[0], (long)enc_last_log[1],
                     (long)enc_last_log[2], (long)enc_last_log[3]);
            return true;
        }
        delay_ms(10);
        elapsed += 10;
    }
    /* 超时 — 停车并告警 */
    Contrl_Speed(0, 0, 0, 0);
    encoder_refresh();
    ESP_LOGW(TAG, "编码器: 超时 %dms 增量=[%ld %ld %ld %ld]",
             elapsed, (long)(Encoder_Now[0] - encoder_base[0]),
             (long)(Encoder_Now[1] - encoder_base[1]),
             (long)(Encoder_Now[2] - encoder_base[2]),
             (long)(Encoder_Now[3] - encoder_base[3]));
    return false;
}

/* 将当前编码器值快照为零点。最多等 100ms 拿一次新读数。 */
static void encoder_snapshot(void)
{
    int w = 0;
    while (!g_recv_flag && w < 100) { delay_ms(10); w += 10; }
    encoder_refresh();
    for (int i = 0; i < 4; i++) encoder_base[i] = Encoder_Now[i];
}

/* ── 电机任务 (编码器闭环) ── */
static void motor_task(void *arg)
{
    motor_msg_t msg;

    while (1) {
        if (xQueueReceive(g_motor_queue, &msg, portMAX_DELAY) != pdTRUE)
            continue;

        /* 速度模式激活时丢弃队列中的位置命令 */
        if (g_velocity_active) {
            ESP_LOGI(TAG, "电机: 队列消息丢弃(速度模式活跃)");
            continue;
        }

        int ms = msg.speed * 10;
        if (ms < 100) ms = 100;
        if (ms > 1000) ms = 1000;

        int32_t dist_pulses = cm_to_pulses(msg.distance_cm);
        int32_t t[4] = {0};
        int timeout = msg.distance_cm * ENC_TIMEOUT_PER_CM + 5000;

        ESP_LOGI(TAG, "电机: cmd=%d 距离=%dcm(%ld脉冲) 速度=%d 超时=%dms",
                 msg.cmd, msg.distance_cm, (long)dist_pulses, msg.speed, timeout);

        g_motor_stop_flag = false;

        switch (msg.cmd) {
        case MOTOR_CMD_GO: {
            encoder_snapshot();
            t[0] = t[1] = t[2] = t[3] = dist_pulses;
            Contrl_Speed(ms, ms, ms, ms);
            motor_encoder_wait(t, timeout);
            g_position_active = false;
            break;
        }
        default:
            break;
        }
    }
}

/* ── 电机控制回调 (来自 HTTP) ── */
static void motor_control_callback(motor_cmd_t cmd, int distance_cm, int speed)
{
    if (cmd == MOTOR_CMD_STOP) {
        /* 立即停车 — 取消速度及位置模式 */
        g_motor_stop_flag = true;
        g_velocity_active = false;
        g_position_active = false;
        Contrl_Speed(0, 0, 0, 0);
        motor_msg_t junk;
        while (xQueueReceive(g_motor_queue, &junk, 0) == pdTRUE) { }
        ESP_LOGI(TAG, "电机: 停止 (立即)");
        return;
    }

    /* ── 速度模式 (D-Pad): 直接调速，不用编码器 ── */
    if (cmd == MOTOR_CMD_VEL_FWD || cmd == MOTOR_CMD_VEL_BACK ||
        cmd == MOTOR_CMD_VEL_LEFT || cmd == MOTOR_CMD_VEL_RIGHT) {

        /* 取消任何待执行的位置移动 */
        g_motor_stop_flag = true;
        g_velocity_active = true;
        g_position_active = false;
        motor_msg_t junk;
        while (xQueueReceive(g_motor_queue, &junk, 0) == pdTRUE) { }

        /* 短暂延迟让 motor_task 退出 encoder_wait */
        delay_ms(20);

        int16_t ms = (int16_t)(speed * 10);
        if (ms < 100) ms = 100;
        if (ms > 600) ms = 600;  /* 速度模式限速，安全第一 */

        switch (cmd) {
        case MOTOR_CMD_VEL_FWD:
            Contrl_Speed(ms, ms, ms, ms);
            ESP_LOGI(TAG, "电机: 前进 速度=%d", ms);
            break;
        case MOTOR_CMD_VEL_BACK:
            Contrl_Speed(-ms, -ms, -ms, -ms);
            ESP_LOGI(TAG, "电机: 后退 速度=%d", ms);
            break;
        case MOTOR_CMD_VEL_LEFT:
            Contrl_Speed(-ms, -ms, ms, ms);
            ESP_LOGI(TAG, "电机: 左转 速度=%d", ms);
            break;
        case MOTOR_CMD_VEL_RIGHT:
            Contrl_Speed(ms, ms, -ms, -ms);
            ESP_LOGI(TAG, "电机: 右转 速度=%d", ms);
            break;
        default: break;
        }
        return;
    }

    /* ── 位置模式 (GO 按钮): 编码器闭环 ── */
    if (cmd == MOTOR_CMD_GO) {
        g_velocity_active = false;
        g_position_active = true;
        motor_msg_t msg = { .cmd = cmd, .distance_cm = distance_cm, .speed = speed };
        if (xQueueSend(g_motor_queue, &msg, 0) != pdTRUE) {
            Contrl_Speed(0, 0, 0, 0);
            g_position_active = false;
            ESP_LOGW(TAG, "电机队列满，强制停车");
        }
        return;
    }

    /* 旧命令 (MOTOR_CMD_FORWARD 等) — 当作 GO 处理 */
    g_velocity_active = false;
    motor_msg_t msg = { .cmd = cmd, .distance_cm = distance_cm, .speed = speed };
    if (xQueueSend(g_motor_queue, &msg, 0) != pdTRUE) {
        Contrl_Speed(0, 0, 0, 0);
        ESP_LOGW(TAG, "电机队列满，强制停车");
    }
}

/* ── 电机初始化 ── */
static void motor_init(void)
{
    ESP_LOGI(TAG, "Init UART0 (TX=%d RX=%d 115200)...", UART0_TX_PIN, UART0_RX_PIN);
    uart0_init();
    delay_ms(100);

    printf("please wait...\r\n");

    /* 先关闭上传 */
    send_upload_data(false, false, false);
    delay_ms(10);

    /* 配置电机参数 */
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

    /* 开启编码器上传 */
#if UPLOAD_DATA == 1
    send_upload_data(true, false, false);
#elif UPLOAD_DATA == 2
    send_upload_data(false, true, false);
#elif UPLOAD_DATA == 3
    send_upload_data(false, false, true);
#endif
    delay_ms(10);

    /* 启动 UART 接收任务 */
    xTaskCreate(UART_Process_Task, "UART_RX", 4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "电机类型 %d 已就绪 (UART0 TX=%d RX=%d)", MOTOR_TYPE, UART0_TX_PIN, UART0_RX_PIN);
}

/* ── 编码器监控任务 ── */
static void encoder_monitor_task(void *arg)
{
    while (1) {
        encoder_refresh();
        delay_ms(20);
    }
}

/* ── 主函数 ── */
void app_main(void)
{
    esp_err_t ret;

    /* 抑制 httpd socket 噪音 (ECONNRESET=104 来自手机轮询/断连) */
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
    esp_log_level_set("httpd", ESP_LOG_WARN);
    /* 抑制无害的传感器自动检测失败 (OV5647 等) — NONE 全部静音 */
    esp_log_level_set("ov5647", ESP_LOG_NONE);
    esp_log_level_set("sccb_i2c", ESP_LOG_ERROR);
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);
    /* SDIO 空闲状态事件是无害的 ESP-Hosted 传输噪音 */
    esp_log_level_set("sdmmc_req", ESP_LOG_NONE);

    /* 1. NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    /* 2. CSI 摄像头 — 尽早初始化以提供 XCLK (官方示例模式) */
    ESP_LOGI(TAG, "Initializing CSI camera...");
    esp_err_t cam_err = camera_module_init(NULL);
    if (cam_err != ESP_OK) {
        ESP_LOGW(TAG, "Camera init failed (%s) — running without camera", esp_err_to_name(cam_err));
    }

    /* 3. 电机初始化 — UART0 重新初始化 (拆除控制台 UART，安装带接收队列的驱动) */
    motor_init();

    /* 4. 编码器监控任务 */
    xTaskCreate(encoder_monitor_task, "Enc_Mon", 2048, NULL, 1, NULL);

    /* 5. 电机命令队列 + 任务 — 必须在 web_control_start 之前创建 */
    g_motor_queue = xQueueCreate(5, sizeof(motor_msg_t));
    if (!g_motor_queue) { ESP_LOGE(TAG, "Motor queue create fail"); }
    xTaskCreate(motor_task, "Motor_Task", 4096, NULL, 5, NULL);

    /* 6. WiFi AP */
    ESP_ERROR_CHECK(wifi_module_init_netstack());
    ESP_ERROR_CHECK(wifi_module_init_ap(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL));

    /* 7. IMU USB CDC 主机 — 非阻塞，后台连接任务 */
    ESP_LOGI(TAG, "正在初始化 IMU USB ...");
    esp_err_t imu_err = imu_usb_init();
    if (imu_err != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed (%s) — running without IMU", esp_err_to_name(imu_err));
    }

    /* 8. 网页初始化成功（返回电机引脚） */
    ESP_ERROR_CHECK(web_control_start(motor_control_callback));

    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, " WiFi AP  : %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, " 密码     : %s", WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, " 控制页   : http://192.168.4.1/");
    ESP_LOGI(TAG, " 摄像头   : http://192.168.4.1:81/");
    ESP_LOGI(TAG, " 电机     : UART0 (TX=%d RX=%d) 类型=%d", UART0_TX_PIN, UART0_RX_PIN, MOTOR_TYPE);
    ESP_LOGI(TAG, " 模式     : D-Pad=速度模式 | GO=位置模式");
    ESP_LOGI(TAG, "================================================");

    while (1) { delay_ms(1000); }
}

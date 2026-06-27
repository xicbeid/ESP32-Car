/*
 * WiFi 模块 — ESP32-P4 WiFi 通过 ESP-Hosted (WiFi Remote)
 *
 * 核心实现参考了 esp_brookesia_phone 中
 * components/apps/setting/Setting.cpp 的 WiFi 逻辑，
 * 并移除了所有 LVGL/UI 依赖。
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs.h"
#include "esp_mac.h"
#include "wifi_module.h"

#define TAG "WiFi_Module"

/* ===================== 事件组位标志 ===================== */
#define WIFI_MODULE_CONNECTED_BIT  BIT0
#define WIFI_MODULE_INIT_DONE_BIT  BIT1
#define WIFI_MODULE_SCANNING_BIT   BIT2

/* ===================== 任务配置 ===================== */
#define WIFI_SCAN_TASK_STACK_SIZE   (1024 * 6)
#define WIFI_SCAN_TASK_PRIORITY     (1)
#define WIFI_SCAN_TASK_PERIOD_MS    (5 * 1000)

#define WIFI_CONNECT_TASK_STACK_SIZE (1024 * 4)
#define WIFI_CONNECT_TASK_PRIORITY   (4)
#define WIFI_CONNECT_TIMEOUT_MS      (10 * 1000)

/* ===================== NVS 配置 ===================== */
#define NVS_STORAGE_NAMESPACE       "storage"
#define NVS_KEY_WIFI_ENABLE         "wifi_en"

/* ===================== 扫描配置 ===================== */
#define SCAN_LIST_SIZE              25

/* ===================== 静态全局变量 ===================== */
static EventGroupHandle_t s_wifi_event_group;
static bool s_netstack_initialized = false;

/* 最新扫描结果缓存 */
static wifi_ap_record_t s_ap_info[SCAN_LIST_SIZE];
static uint16_t s_ap_count = 0;

/* 当前连接 SSID / 密码 */
static char s_wifi_ssid[32];
static char s_wifi_password[64];

/* 任务句柄 */
static TaskHandle_t s_scan_task_handle = NULL;

/* ===================== 前向声明 ===================== */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data);
static void wifi_scan_task(void *arg);
static void wifi_connect_task(void *arg);
static bool load_nvs_param(const char *key, int32_t *value);
static bool set_nvs_param(const char *key, int32_t value);

/* ===================== 公开 API ===================== */

esp_err_t wifi_module_init_netstack(void)
{
    if (s_netstack_initialized) {
        ESP_LOGI(TAG, "网络协议栈已初始化");
        return ESP_OK;
    }

    esp_err_t err;

    /* 创建事件组 */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "创建 WiFi 事件组失败");
        return ESP_ERR_NO_MEM;
    }
    xEventGroupClearBits(s_wifi_event_group,
        WIFI_MODULE_CONNECTED_BIT | WIFI_MODULE_INIT_DONE_BIT | WIFI_MODULE_SCANNING_BIT);

    /* 加载 NVS WiFi 使能标志 */
    int32_t wifi_en = 0;
    load_nvs_param(NVS_KEY_WIFI_ENABLE, &wifi_en);
    if (!wifi_en) {
        set_nvs_param(NVS_KEY_WIFI_ENABLE, 1);
    }

    /* 初始化网络协议栈 */
    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init 失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_event_loop_create_default 失败: %s", esp_err_to_name(err));
        return err;
    }

    s_netstack_initialized = true;
    ESP_LOGI(TAG, "网络协议栈已初始化");
    return ESP_OK;
}

esp_err_t wifi_module_init_sta(void)
{
    esp_err_t err;

    if (!s_netstack_initialized) {
        ESP_LOGE(TAG, "网络协议栈未初始化，请先调用 wifi_module_init_netstack()");
        return ESP_ERR_INVALID_STATE;
    }

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    /* 初始化 WiFi 驱动 (ESP-Hosted 会启动从属芯片) */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 注册 WiFi 事件处理 */
    err = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &wifi_event_handler,
                                              NULL,
                                              NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 WiFi 事件处理失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 注册 IP 事件以检测 DHCP 成功 */
    err = esp_event_handler_instance_register(IP_EVENT,
                                              IP_EVENT_STA_GOT_IP,
                                              &wifi_event_handler,
                                              NULL,
                                              NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 IP 事件处理失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 设置模式并启动 WiFi */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode 失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 创建后台扫描任务 */
    BaseType_t ret = xTaskCreate(wifi_scan_task, "WiFi_Scan",
                                  WIFI_SCAN_TASK_STACK_SIZE, NULL,
                                  WIFI_SCAN_TASK_PRIORITY, &s_scan_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 WiFi 扫描任务失败");
        return ESP_ERR_NO_MEM;
    }

    xEventGroupSetBits(s_wifi_event_group, WIFI_MODULE_INIT_DONE_BIT);
    ESP_LOGI(TAG, "WiFi STA 模式已初始化");

    return ESP_OK;
}

esp_err_t wifi_module_init_ap(const char *ssid, const char *password, uint8_t channel)
{
    esp_err_t err;

    if (!s_netstack_initialized) {
        ESP_LOGE(TAG, "网络协议栈未初始化，请先调用 wifi_module_init_netstack()");
        return ESP_ERR_INVALID_STATE;
    }

    if (ssid == NULL) {
        ESP_LOGE(TAG, "SSID 不能为 NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* 创建 AP netif */
    esp_netif_create_default_wifi_ap();

    /* 初始化 WiFi 驱动 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 注册 WiFi 事件处理 */
    err = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &wifi_event_handler,
                                              NULL,
                                              NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 WiFi 事件处理失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 配置 AP */
    wifi_config_t ap_cfg = { 0 };
    strncpy((char *)ap_cfg.ap.ssid, ssid, sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid_len = strlen(ssid);
    ap_cfg.ap.channel = (channel > 0) ? channel : 1;
    ap_cfg.ap.max_connection = 4;

    if (password != NULL && strlen(password) >= 8) {
        strncpy((char *)ap_cfg.ap.password, password, sizeof(ap_cfg.ap.password) - 1);
        ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode AP 失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config AP 失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start AP 失败: %s", esp_err_to_name(err));
        return err;
    }

    s_netstack_initialized = true;
    ESP_LOGI(TAG, "WiFi AP 已启动. SSID: %s 信道: %d", ssid, (channel > 0) ? channel : 1);
    return ESP_OK;
}

/* 兼容旧接口 — 初始化 netstack + STA */
esp_err_t wifi_module_init(void)
{
    esp_err_t err = wifi_module_init_netstack();
    if (err != ESP_OK) return err;
    return wifi_module_init_sta();
}

esp_err_t wifi_module_scan_start(void)
{
    ESP_LOGI(TAG, "WiFi 扫描已启动");
    xEventGroupSetBits(s_wifi_event_group, WIFI_MODULE_SCANNING_BIT);
    return ESP_OK;
}

void wifi_module_scan_stop(void)
{
    ESP_LOGI(TAG, "WiFi 扫描已停止");
    xEventGroupClearBits(s_wifi_event_group, WIFI_MODULE_SCANNING_BIT);
    esp_wifi_scan_stop();
    s_ap_count = 0;
}

uint16_t wifi_module_get_scan_results(wifi_ap_record_t *ap_info, uint16_t max_ap)
{
    uint16_t count = (s_ap_count < max_ap) ? s_ap_count : max_ap;
    if (count > 0 && ap_info != NULL) {
        memcpy(ap_info, s_ap_info, count * sizeof(wifi_ap_record_t));
    }
    return count;
}

esp_err_t wifi_module_connect(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "SSID 或密码无效");
        return ESP_ERR_INVALID_ARG;
    }

    /* 连接期间停止扫描 */
    wifi_module_scan_stop();

    /* 保存 SSID/密码供连接任务使用 */
    strncpy(s_wifi_ssid, ssid, sizeof(s_wifi_ssid) - 1);
    s_wifi_ssid[sizeof(s_wifi_ssid) - 1] = '\0';
    strncpy(s_wifi_password, password, sizeof(s_wifi_password) - 1);
    s_wifi_password[sizeof(s_wifi_password) - 1] = '\0';

    /* 创建连接任务 */
    BaseType_t ret = xTaskCreate(wifi_connect_task, "WiFi_Connect",
                                  WIFI_CONNECT_TASK_STACK_SIZE, NULL,
                                  WIFI_CONNECT_TASK_PRIORITY, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 WiFi 连接任务失败");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t wifi_module_disconnect(void)
{
    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_disconnect 失败: %s", esp_err_to_name(err));
    }
    xEventGroupClearBits(s_wifi_event_group, WIFI_MODULE_CONNECTED_BIT);
    return err;
}

bool wifi_module_is_connected(void)
{
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_MODULE_CONNECTED_BIT) != 0;
}

/* ===================== 事件处理 ===================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi STA 已启动");
            break;

        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "WiFi AP 已启动");
            break;

        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *event =
                (wifi_event_ap_staconnected_t *)event_data;
            ESP_LOGI(TAG, "终端 " MACSTR " 已加入, AID=%d",
                     MAC2STR(event->mac), event->aid);
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *event =
                (wifi_event_ap_stadisconnected_t *)event_data;
            ESP_LOGI(TAG, "终端 " MACSTR " 已离开, AID=%d",
                     MAC2STR(event->mac), event->aid);
            break;
        }

        case WIFI_EVENT_STA_CONNECTED: {
            wifi_event_sta_connected_t *event =
                (wifi_event_sta_connected_t *)event_data;
            ESP_LOGI(TAG, "WiFi 已连接 SSID: %s, BSSID: " MACSTR,
                     event->ssid, MAC2STR(event->bssid));
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *event =
                (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGI(TAG, "WiFi 已断开, 原因: %d", event->reason);
            xEventGroupClearBits(s_wifi_event_group, WIFI_MODULE_CONNECTED_BIT);
            break;
        }

        case WIFI_EVENT_SCAN_DONE:
            ESP_LOGI(TAG, "WiFi 扫描完成");
            break;

        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "获取 IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_MODULE_CONNECTED_BIT);
    }
}

/* ===================== 扫描任务 ===================== */

static void wifi_scan_task(void *arg)
{
    /* 等待 WiFi 初始化完成 */
    xEventGroupWaitBits(s_wifi_event_group, WIFI_MODULE_INIT_DONE_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "WiFi 扫描任务就绪");

    while (1) {
        /* 等待扫描请求 */
        while (!(xEventGroupGetBits(s_wifi_event_group) & WIFI_MODULE_SCANNING_BIT)) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* 执行扫描 */
        uint16_t number = SCAN_LIST_SIZE;
        memset(s_ap_info, 0, sizeof(s_ap_info));

        esp_wifi_scan_start(NULL, true);
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&s_ap_count));
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, s_ap_info));

        ESP_LOGI(TAG, "扫描发现 %u 个 AP", s_ap_count);
        /* 只打印前 5 个 AP 避免串口刷屏 */
        int max_print = (s_ap_count < 5) ? s_ap_count : 5;
        for (int i = 0; i < max_print; i++) {
            ESP_LOGI(TAG, "  [%d] SSID: %-24s RSSI: %4d  认证: %d",
                     i, s_ap_info[i].ssid, s_ap_info[i].rssi, s_ap_info[i].authmode);
        }

        /* 等待下一个扫描周期 */
        vTaskDelay(pdMS_TO_TICKS(WIFI_SCAN_TASK_PERIOD_MS));
    }
}

/* ===================== 连接任务 ===================== */

static void wifi_connect_task(void *arg)
{
    wifi_config_t wifi_config = { 0 };

    /* 连接前先断开 */
    esp_wifi_disconnect();

    memcpy(wifi_config.sta.ssid, s_wifi_ssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, s_wifi_password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_LOGI(TAG, "正在连接 SSID: %s", wifi_config.sta.ssid);
    esp_wifi_connect();

    /* 等待连接结果 */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_MODULE_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_MODULE_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi 连接成功");
        set_nvs_param(NVS_KEY_WIFI_ENABLE, 1);
    } else {
        ESP_LOGI(TAG, "WiFi 连接失败 (超时或密码错误)");
    }

    vTaskDelete(NULL);
}

/* ===================== NVS 辅助函数 ===================== */

static bool load_nvs_param(const char *key, int32_t *value)
{
    esp_err_t err;
    nvs_handle_t nvs_handle;

    err = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "打开 NVS 失败: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_get_i32(nvs_handle, key, value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "NVS 键 '%s' 未找到，使用默认值", key);
        /* 写入默认值 */
        nvs_set_i32(nvs_handle, key, *value);
        nvs_commit(nvs_handle);
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "读取 NVS 键 '%s' 失败: %s", key, esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    nvs_close(nvs_handle);
    return true;
}

static bool set_nvs_param(const char *key, int32_t value)
{
    esp_err_t err;
    nvs_handle_t nvs_handle;

    err = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "打开 NVS 失败: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_i32(nvs_handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置 NVS 键 '%s' 失败: %s", key, esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "提交 NVS 失败: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return (err == ESP_OK);
}

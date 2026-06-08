/*
 * WiFi Module - ESP32-P4 WiFi via ESP-Hosted (WiFi Remote)
 *
 * Core implementation referencing the WiFi logic from
 * esp_brookesia_phone/components/apps/setting/Setting.cpp
 * with all LVGL/UI dependencies removed.
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

/* ===================== Event Group Bits ===================== */
#define WIFI_MODULE_CONNECTED_BIT  BIT0
#define WIFI_MODULE_INIT_DONE_BIT  BIT1
#define WIFI_MODULE_SCANNING_BIT   BIT2

/* ===================== Task Config ===================== */
#define WIFI_SCAN_TASK_STACK_SIZE   (1024 * 6)
#define WIFI_SCAN_TASK_PRIORITY     (1)
#define WIFI_SCAN_TASK_PERIOD_MS    (5 * 1000)

#define WIFI_CONNECT_TASK_STACK_SIZE (1024 * 4)
#define WIFI_CONNECT_TASK_PRIORITY   (4)
#define WIFI_CONNECT_TIMEOUT_MS      (10 * 1000)

/* ===================== NVS Config ===================== */
#define NVS_STORAGE_NAMESPACE       "storage"
#define NVS_KEY_WIFI_ENABLE         "wifi_en"

/* ===================== Scan Config ===================== */
#define SCAN_LIST_SIZE              25

/* ===================== Static Globals ===================== */
static EventGroupHandle_t s_wifi_event_group;
static bool s_netstack_initialized = false;

/* latest scan result cache */
static wifi_ap_record_t s_ap_info[SCAN_LIST_SIZE];
static uint16_t s_ap_count = 0;

/* current connection SSID / password */
static char s_wifi_ssid[32];
static char s_wifi_password[64];

/* task handles */
static TaskHandle_t s_scan_task_handle = NULL;

/* ===================== Forward Declarations ===================== */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data);
static void wifi_scan_task(void *arg);
static void wifi_connect_task(void *arg);
static bool load_nvs_param(const char *key, int32_t *value);
static bool set_nvs_param(const char *key, int32_t value);

/* ===================== Public API ===================== */

esp_err_t wifi_module_init_netstack(void)
{
    if (s_netstack_initialized) {
        ESP_LOGI(TAG, "Networking stack already initialized");
        return ESP_OK;
    }

    esp_err_t err;

    /* create event group */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return ESP_ERR_NO_MEM;
    }
    xEventGroupClearBits(s_wifi_event_group,
        WIFI_MODULE_CONNECTED_BIT | WIFI_MODULE_INIT_DONE_BIT | WIFI_MODULE_SCANNING_BIT);

    /* load NVS WiFi enable flag */
    int32_t wifi_en = 0;
    load_nvs_param(NVS_KEY_WIFI_ENABLE, &wifi_en);
    if (!wifi_en) {
        set_nvs_param(NVS_KEY_WIFI_ENABLE, 1);
    }

    /* init networking stack */
    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return err;
    }

    s_netstack_initialized = true;
    ESP_LOGI(TAG, "Networking stack initialized");
    return ESP_OK;
}

esp_err_t wifi_module_init_sta(void)
{
    esp_err_t err;

    if (!s_netstack_initialized) {
        ESP_LOGE(TAG, "Netstack not initialized, call wifi_module_init_netstack() first");
        return ESP_ERR_INVALID_STATE;
    }

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    /* init WiFi driver (ESP-Hosted will bring up the slave chip) */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* register WiFi event handler */
    err = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &wifi_event_handler,
                                              NULL,
                                              NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Register WiFi event handler failed: %s", esp_err_to_name(err));
        return err;
    }

    /* register IP events to detect successful DHCP */
    err = esp_event_handler_instance_register(IP_EVENT,
                                              IP_EVENT_STA_GOT_IP,
                                              &wifi_event_handler,
                                              NULL,
                                              NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Register IP event handler failed: %s", esp_err_to_name(err));
        return err;
    }

    /* set mode and start WiFi */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        return err;
    }

    /* create background scan task */
    BaseType_t ret = xTaskCreate(wifi_scan_task, "WiFi_Scan",
                                  WIFI_SCAN_TASK_STACK_SIZE, NULL,
                                  WIFI_SCAN_TASK_PRIORITY, &s_scan_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create WiFi scan task");
        return ESP_ERR_NO_MEM;
    }

    xEventGroupSetBits(s_wifi_event_group, WIFI_MODULE_INIT_DONE_BIT);
    ESP_LOGI(TAG, "WiFi STA mode initialized");

    return ESP_OK;
}

esp_err_t wifi_module_init_ap(const char *ssid, const char *password, uint8_t channel)
{
    esp_err_t err;

    if (!s_netstack_initialized) {
        ESP_LOGE(TAG, "Netstack not initialized, call wifi_module_init_netstack() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (ssid == NULL) {
        ESP_LOGE(TAG, "SSID cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* create AP netif */
    esp_netif_create_default_wifi_ap();

    /* init WiFi driver */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* register WiFi event handler */
    err = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &wifi_event_handler,
                                              NULL,
                                              NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Register WiFi event handler failed: %s", esp_err_to_name(err));
        return err;
    }

    /* configure AP */
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
        ESP_LOGE(TAG, "esp_wifi_set_mode AP failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config AP failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start AP failed: %s", esp_err_to_name(err));
        return err;
    }

    s_netstack_initialized = true;
    ESP_LOGI(TAG, "WiFi AP started. SSID: %s channel: %d", ssid, (channel > 0) ? channel : 1);
    return ESP_OK;
}

/* Legacy wrapper — init netstack + STA, for backward compatibility */
esp_err_t wifi_module_init(void)
{
    esp_err_t err = wifi_module_init_netstack();
    if (err != ESP_OK) return err;
    return wifi_module_init_sta();
}

esp_err_t wifi_module_scan_start(void)
{
    ESP_LOGI(TAG, "WiFi scan started");
    xEventGroupSetBits(s_wifi_event_group, WIFI_MODULE_SCANNING_BIT);
    return ESP_OK;
}

void wifi_module_scan_stop(void)
{
    ESP_LOGI(TAG, "WiFi scan stopped");
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
        ESP_LOGE(TAG, "Invalid SSID or password");
        return ESP_ERR_INVALID_ARG;
    }

    /* stop scanning while connecting */
    wifi_module_scan_stop();

    /* save SSID/password for the connect task */
    strncpy(s_wifi_ssid, ssid, sizeof(s_wifi_ssid) - 1);
    s_wifi_ssid[sizeof(s_wifi_ssid) - 1] = '\0';
    strncpy(s_wifi_password, password, sizeof(s_wifi_password) - 1);
    s_wifi_password[sizeof(s_wifi_password) - 1] = '\0';

    /* create connect task */
    BaseType_t ret = xTaskCreate(wifi_connect_task, "WiFi_Connect",
                                  WIFI_CONNECT_TASK_STACK_SIZE, NULL,
                                  WIFI_CONNECT_TASK_PRIORITY, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create WiFi connect task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t wifi_module_disconnect(void)
{
    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_disconnect failed: %s", esp_err_to_name(err));
    }
    xEventGroupClearBits(s_wifi_event_group, WIFI_MODULE_CONNECTED_BIT);
    return err;
}

bool wifi_module_is_connected(void)
{
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_MODULE_CONNECTED_BIT) != 0;
}

/* ===================== Event Handler ===================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi STA started");
            break;

        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "WiFi AP started");
            break;

        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *event =
                (wifi_event_ap_staconnected_t *)event_data;
            ESP_LOGI(TAG, "Station " MACSTR " joined, AID=%d",
                     MAC2STR(event->mac), event->aid);
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *event =
                (wifi_event_ap_stadisconnected_t *)event_data;
            ESP_LOGI(TAG, "Station " MACSTR " left, AID=%d",
                     MAC2STR(event->mac), event->aid);
            break;
        }

        case WIFI_EVENT_STA_CONNECTED: {
            wifi_event_sta_connected_t *event =
                (wifi_event_sta_connected_t *)event_data;
            ESP_LOGI(TAG, "WiFi connected to SSID: %s, BSSID: " MACSTR,
                     event->ssid, MAC2STR(event->bssid));
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *event =
                (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGI(TAG, "WiFi disconnected, reason: %d", event->reason);
            xEventGroupClearBits(s_wifi_event_group, WIFI_MODULE_CONNECTED_BIT);
            break;
        }

        case WIFI_EVENT_SCAN_DONE:
            ESP_LOGI(TAG, "WiFi scan done");
            break;

        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_MODULE_CONNECTED_BIT);
    }
}

/* ===================== Scan Task ===================== */

static void wifi_scan_task(void *arg)
{
    /* wait until WiFi init is done */
    xEventGroupWaitBits(s_wifi_event_group, WIFI_MODULE_INIT_DONE_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "WiFi scan task ready");

    while (1) {
        /* wait for scanning to be requested */
        while (!(xEventGroupGetBits(s_wifi_event_group) & WIFI_MODULE_SCANNING_BIT)) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* do the scan */
        uint16_t number = SCAN_LIST_SIZE;
        memset(s_ap_info, 0, sizeof(s_ap_info));

        esp_wifi_scan_start(NULL, true);
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&s_ap_count));
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, s_ap_info));

        ESP_LOGI(TAG, "Scan found %u APs", s_ap_count);
        /* Only print first 5 APs to avoid serial flooding */
        int max_print = (s_ap_count < 5) ? s_ap_count : 5;
        for (int i = 0; i < max_print; i++) {
            ESP_LOGI(TAG, "  [%d] SSID: %-24s RSSI: %4d  Auth: %d",
                     i, s_ap_info[i].ssid, s_ap_info[i].rssi, s_ap_info[i].authmode);
        }

        /* wait for next scan period */
        vTaskDelay(pdMS_TO_TICKS(WIFI_SCAN_TASK_PERIOD_MS));
    }
}

/* ===================== Connect Task ===================== */

static void wifi_connect_task(void *arg)
{
    wifi_config_t wifi_config = { 0 };

    /* disconnect before connecting */
    esp_wifi_disconnect();

    memcpy(wifi_config.sta.ssid, s_wifi_ssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, s_wifi_password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_LOGI(TAG, "Connecting to SSID: %s", wifi_config.sta.ssid);
    esp_wifi_connect();

    /* wait for connection result */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_MODULE_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_MODULE_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected successfully");
        set_nvs_param(NVS_KEY_WIFI_ENABLE, 1);
    } else {
        ESP_LOGI(TAG, "WiFi connection failed (timeout or wrong password)");
    }

    vTaskDelete(NULL);
}

/* ===================== NVS Helpers ===================== */

static bool load_nvs_param(const char *key, int32_t *value)
{
    esp_err_t err;
    nvs_handle_t nvs_handle;

    err = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_get_i32(nvs_handle, key, value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "NVS key '%s' not found, using default", key);
        /* write default */
        nvs_set_i32(nvs_handle, key, *value);
        nvs_commit(nvs_handle);
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading NVS key '%s': %s", key, esp_err_to_name(err));
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
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_i32(nvs_handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error setting NVS key '%s': %s", key, esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return (err == ESP_OK);
}

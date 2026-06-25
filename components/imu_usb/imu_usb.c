/*
 * IMU USB CDC Host — Wit-Motion 10-Axis IMU via CP2102/CH340/FTDI
 *
 * USB Host init → CDC ACM driver install → open device → receive bytes
 * → feed parser → thread-safe data to web_control.
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "imu_parser.h"
#include "imu_usb.h"

static const char *TAG = "IMU-USB";

/* ── Known USB-UART bridge VID:PID pairs ──────────────────────── */
typedef struct {
    uint16_t vid;
    uint16_t pid;
    const char *name;
} usb_device_id_t;

static const usb_device_id_t s_known_devices[] = {
    { 0x10C4, 0xEA60, "CP2102" },
    { 0x10C4, 0xEA70, "CP210x" },
    { 0x1A86, 0x7523, "CH340"  },
    { 0x1A86, 0x55D3, "CH343"  },
    { 0x0403, 0x6001, "FT232"  },
    { 0x0403, 0x6015, "FT231"  },
};

/* ── State ────────────────────────────────────────────────────── */
static SemaphoreHandle_t  s_data_mutex  = NULL;
static SemaphoreHandle_t  s_disconnect_sem = NULL;
static imu_data_t         s_imu_data;
static volatile bool      s_connected   = false;
static cdc_acm_dev_hdl_t  s_dev_hdl     = NULL;

/* ── CDC ACM data receive callback ────────────────────────────── */
static bool cdc_rx_cb(const uint8_t *buf, size_t len, void *arg)
{
    for (size_t i = 0; i < len; i++) {
        imu_parser_feed(buf[i]);
    }
    return true;
}

/* ── IMU parser data-ready callback ───────────────────────────── */
static void on_imu_frame(const imu_data_t *data, uint8_t type)
{
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_imu_data = *data;
        xSemaphoreGive(s_data_mutex);
    }
    (void)type;
}

/* ── CDC ACM device event callback ────────────────────────────── */
static void cdc_event_cb(const cdc_acm_host_dev_event_data_t *event,
                         void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC error, err_no=%i", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "IMU disconnected!");
        cdc_acm_host_close(event->data.cdc_hdl);
        s_connected = false;
        s_dev_hdl = NULL;
        xSemaphoreGive(s_disconnect_sem);
        break;
    default:
        break;
    }
}

/* ── USB host library event task ──────────────────────────────── */
static void usb_lib_task(void *arg)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: all devices freed");
        }
    }
}

/* ── Try open known devices ───────────────────────────────────── */
static esp_err_t try_open_device(const cdc_acm_host_device_config_t *cfg,
                                  cdc_acm_dev_hdl_t *out_hdl)
{
    for (int i = 0; i < sizeof(s_known_devices)/sizeof(s_known_devices[0]); i++) {
        ESP_LOGI(TAG, "Try %s (0x%04X:0x%04X) ...",
                 s_known_devices[i].name,
                 s_known_devices[i].vid,
                 s_known_devices[i].pid);
        esp_err_t err = cdc_acm_host_open(s_known_devices[i].vid,
                                           s_known_devices[i].pid,
                                           0, cfg, out_hdl);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Device opened: %s", s_known_devices[i].name);
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

/* ── Connection task ──────────────────────────────────────────── */
static void imu_connect_task(void *arg)
{
    const cdc_acm_host_device_config_t dev_cfg = {
        .connection_timeout_ms = 2000,
        .out_buffer_size = 64,
        .in_buffer_size = 512,
        .user_arg = NULL,
        .event_cb = cdc_event_cb,
        .data_cb = cdc_rx_cb,
    };

    while (1) {
        cdc_acm_dev_hdl_t dev_hdl = NULL;

        ESP_LOGI(TAG, "Scanning for IMU device ...");
        esp_err_t err = try_open_device(&dev_cfg, &dev_hdl);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "No IMU found. Retrying in 3s ...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        /* ── Try to set 9600 8N1 (silently skip if not supported) ── */
        cdc_acm_line_coding_t lc;
        if (cdc_acm_host_line_coding_get(dev_hdl, &lc) == ESP_OK) {
            lc.dwDTERate = 9600;
            lc.bDataBits = 8;
            lc.bParityType = 0;
            lc.bCharFormat = 1;
            cdc_acm_host_line_coding_set(dev_hdl, &lc);
        }
        cdc_acm_host_set_control_line_state(dev_hdl, true, false);

        imu_parser_reset();
        s_connected = true;
        s_dev_hdl = dev_hdl;
        ESP_LOGI(TAG, "IMU connected — streaming data");

        /* Wait for disconnect */
        xSemaphoreTake(s_disconnect_sem, portMAX_DELAY);
        s_connected = false;
        s_dev_hdl = NULL;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ═══════════════════════════════════════════════════════════════ *
 *  Public API
 * ═══════════════════════════════════════════════════════════════ */

esp_err_t imu_usb_init(void)
{
    ESP_LOGI(TAG, "Initialising IMU USB CDC Host ...");

    /* Create mutex + semaphore */
    s_data_mutex = xSemaphoreCreateMutex();
    s_disconnect_sem = xSemaphoreCreateBinary();
    if (!s_data_mutex || !s_disconnect_sem) {
        ESP_LOGE(TAG, "Failed to create sync objects");
        return ESP_ERR_NO_MEM;
    }

    /* Init parser */
    imu_parser_init(on_imu_frame);

    /* Install USB Host driver */
    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));

    /* USB library event handling task */
    BaseType_t ret = xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 20, NULL);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create usb_lib task");
        return ESP_ERR_NO_MEM;
    }

    /* Install CDC-ACM class driver */
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    /* Start connection task */
    ret = xTaskCreate(imu_connect_task, "imu_conn", 4096, NULL, 10, NULL);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create imu_conn task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "IMU USB Host initialised");
    return ESP_OK;
}

void imu_usb_get_data(imu_data_t *out)
{
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        *out = s_imu_data;
        s_imu_data.updated = 0;  /* consumer clears */
        xSemaphoreGive(s_data_mutex);
    } else {
        memset(out, 0, sizeof(*out));
    }
}

bool imu_usb_is_connected(void)
{
    return s_connected;
}

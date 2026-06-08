/*
 * WiFi Module - ESP32-P4 WiFi via ESP-Hosted (WiFi Remote)
 *
 * This module provides WiFi STA functionality for ESP32-P4, which has no
 * built-in WiFi hardware. It uses the ESP-Hosted protocol (esp_wifi_remote)
 * to communicate with a slave chip (ESP32-C6) over SDIO.
 *
 * API reference: esp_brookesia_phone/components/apps/setting/Setting.cpp
 */

#ifndef __WIFI_MODULE_H__
#define __WIFI_MODULE_H__

#include "esp_err.h"
#include "esp_wifi_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the WiFi networking stack (call once).
 *
 * Initializes netif and event loop. Must be called before
 * wifi_module_init_sta() or wifi_module_init_ap().
 * NVS must be initialized first by the caller.
 *
 * @return ESP_OK on success, otherwise error.
 */
esp_err_t wifi_module_init_netstack(void);

/**
 * @brief Initialize WiFi STA mode via ESP-Hosted.
 *
 * Initializes WiFi driver and starts STA mode. The ESP-Hosted slave
 * will be reset and brought up as part of this process.
 * wifi_module_init_netstack() must be called first.
 *
 * @return ESP_OK on success, otherwise error.
 */
esp_err_t wifi_module_init_sta(void);

/**
 * @brief Initialize WiFi SoftAP mode via ESP-Hosted.
 *
 * Creates a WiFi hotspot that phones can connect to directly.
 * wifi_module_init_netstack() must be called first.
 *
 * @param ssid      WiFi hotspot name (max 32 chars).
 * @param password  WiFi password (min 8 chars, NULL or "" for open).
 * @param channel   WiFi channel (1-13).
 * @return ESP_OK on success, otherwise error.
 */
esp_err_t wifi_module_init_ap(const char *ssid, const char *password, uint8_t channel);

/**
 * @brief Start scanning for nearby WiFi APs.
 *
 * Scanning runs asynchronously in a background FreeRTOS task.
 * Results can be retrieved via wifi_module_get_scan_results().
 *
 * @return ESP_OK on success, otherwise error.
 */
esp_err_t wifi_module_scan_start(void);

/**
 * @brief Stop WiFi scanning and hide results.
 */
void wifi_module_scan_stop(void);

/**
 * @brief Get the latest scan results.
 *
 * @param[out] ap_info   Buffer to hold AP records.
 * @param[in]  max_ap    Maximum number of APs the buffer can hold.
 * @return Number of APs found (0 if none).
 */
uint16_t wifi_module_get_scan_results(wifi_ap_record_t *ap_info, uint16_t max_ap);

/**
 * @brief Connect to the given AP (non-blocking).
 *
 * A FreeRTOS task is spawned to handle the connection.
 * Use wifi_module_is_connected() to check the result.
 *
 * @param ssid      Target AP SSID.
 * @param password  Target AP password.
 * @return ESP_OK if the connect task was created successfully.
 */
esp_err_t wifi_module_connect(const char *ssid, const char *password);

/**
 * @brief Disconnect from the currently connected AP.
 *
 * @return ESP_OK on success.
 */
esp_err_t wifi_module_disconnect(void);

/**
 * @brief Query whether WiFi is currently connected.
 *
 * @return true if connected, false otherwise.
 */
bool wifi_module_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* __WIFI_MODULE_H__ */

/*
 * WiFi 模块 — ESP32-P4 WiFi 通过 ESP-Hosted (WiFi Remote)
 *
 * 本模块为 ESP32-P4 提供 WiFi STA 功能，P4 芯片自身没有
 * 内置 WiFi 硬件。它使用 ESP-Hosted 协议 (esp_wifi_remote)
 * 通过 SDIO 与从属芯片 (ESP32-C6) 通信。
 *
 * API 参考: esp_brookesia_phone/components/apps/setting/Setting.cpp
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
 * @brief 初始化 WiFi 网络协议栈 (仅需调用一次)。
 *
 * 初始化 netif 和事件循环。必须在
 * wifi_module_init_sta() 或 wifi_module_init_ap() 之前调用。
 * 调用者需先初始化 NVS。
 *
 * @return 成功返回 ESP_OK，否则返回错误码。
 */
esp_err_t wifi_module_init_netstack(void);

/**
 * @brief 通过 ESP-Hosted 初始化 WiFi STA 模式。
 *
 * 初始化 WiFi 驱动并启动 STA 模式。
 * ESP-Hosted 从属芯片将在此过程中被复位并启动。
 * 必须首先调用 wifi_module_init_netstack()。
 *
 * @return 成功返回 ESP_OK，否则返回错误码。
 */
esp_err_t wifi_module_init_sta(void);

/**
 * @brief 通过 ESP-Hosted 初始化 WiFi SoftAP 模式。
 *
 * 创建手机可直接连接的 WiFi 热点。
 * 必须首先调用 wifi_module_init_netstack()。
 *
 * @param ssid      WiFi 热点名称 (最大 32 字符)。
 * @param password  WiFi 密码 (最少 8 字符，NULL 或 "" 表示开放网络)。
 * @param channel   WiFi 信道 (1-13)。
 * @return 成功返回 ESP_OK，否则返回错误码。
 */
esp_err_t wifi_module_init_ap(const char *ssid, const char *password, uint8_t channel);

/**
 * @brief 开始扫描附近的 WiFi AP。
 *
 * 扫描在后台 FreeRTOS 任务中异步运行。
 * 结果可通过 wifi_module_get_scan_results() 获取。
 *
 * @return 成功返回 ESP_OK，否则返回错误码。
 */
esp_err_t wifi_module_scan_start(void);

/**
 * @brief 停止 WiFi 扫描并隐藏结果。
 */
void wifi_module_scan_stop(void);

/**
 * @brief 获取最新扫描结果。
 *
 * @param[out] ap_info   存放 AP 记录的缓冲区。
 * @param[in]  max_ap    缓冲区可容纳的最大 AP 数量。
 * @return 找到的 AP 数量 (0 表示无)。
 */
uint16_t wifi_module_get_scan_results(wifi_ap_record_t *ap_info, uint16_t max_ap);

/**
 * @brief 连接到指定 AP (非阻塞)。
 *
 * 会创建一个 FreeRTOS 任务来处理连接。
 * 使用 wifi_module_is_connected() 检查连接结果。
 *
 * @param ssid      目标 AP 的 SSID。
 * @param password  目标 AP 的密码。
 * @return 连接任务创建成功则返回 ESP_OK。
 */
esp_err_t wifi_module_connect(const char *ssid, const char *password);

/**
 * @brief 断开当前连接的 AP。
 *
 * @return 成功返回 ESP_OK。
 */
esp_err_t wifi_module_disconnect(void);

/**
 * @brief 查询 WiFi 是否已连接。
 *
 * @return 已连接返回 true，否则返回 false。
 */
bool wifi_module_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* __WIFI_MODULE_H__ */

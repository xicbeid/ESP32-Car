/*
 * IMU USB CDC 主机 — 公开 API
 */

#pragma once

#include "esp_err.h"
#include "imu_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 USB CDC 主机并开始接收 IMU 数据。
 *
 * 打开 CDC ACM 设备 (CP2102/CH340/FTDI)，波特率 9600 8N1，
 * 解析 Wit-Motion 标准协议数据流。
 *
 * @return 成功返回 ESP_OK
 */
esp_err_t imu_usb_init(void);

/**
 * @brief 获取最新滤波后的 IMU 数据快照。
 *
 * 线程安全。IMU 未连接时返回全零数据。
 *
 * @param out  指向 imu_data_t 的指针，用于填充数据。
 */
void imu_usb_get_data(imu_data_t *out);

/**
 * @brief 检查 IMU 是否已连接并在传输数据。
 */
bool imu_usb_is_connected(void);

#ifdef __cplusplus
}
#endif

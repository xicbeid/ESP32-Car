/*
 * IMU USB CDC Host — Public API
 */

#pragma once

#include "esp_err.h"
#include "imu_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise USB CDC host and start receiving IMU data.
 *
 * Opens a CDC ACM device (CP2102/CH340/FTDI) at 9600 8N1,
 * parses the Wit-Motion Normal Protocol stream.
 *
 * @return ESP_OK on success
 */
esp_err_t imu_usb_init(void);

/**
 * @brief Get the latest filtered IMU data snapshot.
 *
 * Thread-safe. Returns zero-filled data if IMU not connected yet.
 *
 * @param out  Pointer to imu_data_t to fill.
 */
void imu_usb_get_data(imu_data_t *out);

/**
 * @brief Check whether the IMU is connected and streaming.
 */
bool imu_usb_is_connected(void);

#ifdef __cplusplus
}
#endif

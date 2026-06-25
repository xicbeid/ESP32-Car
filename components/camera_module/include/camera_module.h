/*
 * Camera Module — MIPI-CSI camera capture + JPEG encode
 *
 * References:
 *   esp-idf/examples/peripherals/camera/camera_dsi/ (CSI + ISP pipeline)
 *   esp_driver_jpeg (hardware JPEG encoder)
 *   esp_cam_sensor  (auto-detect SC2336/OV5647)
 *
 * Data flow: SC2336 → MIPI-CSI 2-lane RAW8 → CSI Ctrl → ISP → RGB565 → JPEG → HTTP
 *
 * Usage:
 *   1. wifi_module_init_netstack() + wifi_module_init_ap()
 *   2. camera_module_init()
 *   3. Get frames via camera_module_get_frame() for HTTP streaming
 */

#ifndef __CAMERA_MODULE_H__
#define __CAMERA_MODULE_H__

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Camera frame resolution configuration
 */
typedef struct {
    uint16_t width;    /* horizontal pixels (must be 16-aligned for CSI) */
    uint16_t height;   /* vertical pixels */
    uint8_t  quality;  /* JPEG quality 1-100, default 60 */
} camera_config_t;

/**
 * @brief Initialize the camera subsystem.
 *
 * This function:
 *   - Initializes I2C/SCCB bus for sensor control
 *   - Auto-detects the connected camera sensor (SC2336, OV5647, etc.)
 *   - Configures MIPI-CSI controller (2-lane, RAW8→RGB565)
 *   - Initializes ISP processor (RAW8→RGB565 conversion)
 *   - Initializes hardware JPEG encoder
 *   - Allocates frame buffers in PSRAM
 *   - Starts the background capture + encode task
 *
 * Must be called after WiFi and NVS are initialized.
 *
 * @param[in] cfg  Camera configuration (pass NULL for defaults: 800×640, Q=60).
 * @return ESP_OK on success.
 */
esp_err_t camera_module_init(const camera_config_t *cfg);

/**
 * @brief Get the latest JPEG-encoded camera frame.
 *
 * Thread-safe — can be called from HTTP handler task.
 * Returns a pointer to an internal buffer. The data remains valid until
 * the next frame is captured (typically 20-30 ms).
 *
 * @param[out] jpeg_buf  Pointer to JPEG frame data (read-only, do not free).
 * @param[out] jpeg_len  Size of JPEG data in bytes.
 * @return ESP_OK if a frame is available, ESP_ERR_NOT_FOUND if no frame yet.
 */
esp_err_t camera_module_get_frame(const uint8_t **jpeg_buf, size_t *jpeg_len);

/**
 * @brief Start camera streaming (called automatically by _init).
 * @return ESP_OK on success.
 */
esp_err_t camera_module_start(void);

/**
 * @brief Stop camera streaming.
 * @return ESP_OK on success.
 */
esp_err_t camera_module_stop(void);

/**
 * @brief Get the current camera frame rate (smoothed over ~1s window).
 * @return Frames per second.
 */
uint32_t camera_module_get_fps(void);

/**
 * @brief Export the current auto-exposure state.
 * @param[out] brightness  Smoothed brightness value (0.0 - 255.0).
 * @param[out] exp_100us   Exposure time in 100-microsecond units.
 * @param[out] gain_idx    Sensor gain register index.
 */
void camera_module_get_ae_status(float *brightness, int32_t *exp_100us, int32_t *gain_idx);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MODULE_H__ */

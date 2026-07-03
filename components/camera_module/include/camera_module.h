/*
 * 摄像头模块 — MIPI-CSI 摄像头采集 + JPEG 编码
 *
 * 参考:
 *   esp-idf/examples/peripherals/camera/camera_dsi/ (CSI + ISP 管线)
 *   esp_driver_jpeg (硬件 JPEG 编码器)
 *   esp_cam_sensor  (自动检测 SC2336/OV5647)
 *
 * 数据流: SC2336 → MIPI-CSI 2-lane RAW8 → CSI Ctrl → ISP → RGB565 → JPEG → HTTP
 *
 * 使用方式:
 *   1. wifi_module_init_netstack() + wifi_module_init_ap()
 *   2. camera_module_init()
 *   3. 通过 camera_module_get_frame() 获取帧数据用于 HTTP 推流
 */

#ifndef __CAMERA_MODULE_H__
#define __CAMERA_MODULE_H__

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Pre-JPEG 回调 (用于在编码前画检测框等叠加层) ── */
/**
 * @brief 在 JPEG 编码前调用的回调。
 *  仅在 RGB565 模式下有效。回调可原地修改帧数据以叠加图形。
 * @param rgb565  RGB565 帧缓冲区 (可修改)
 * @param w       帧宽度 (像素)
 * @param h       帧高度 (像素)
 * @param stride  行字节数 (通常 w*2)
 * @param user_ctx  用户上下文
 */
typedef void (*camera_pre_jpeg_cb_t)(uint8_t *rgb565, int w, int h,
                                     int stride, void *user_ctx);

/**
 * @brief 注册 pre-JPEG 回调。传 NULL 取消。
 */
void camera_module_set_pre_jpeg_cb(camera_pre_jpeg_cb_t cb, void *user_ctx);

/**
 * @brief 摄像头帧分辨率配置
 */
typedef struct {
    uint16_t width;    /* 水平像素 (CSI 要求 16 对齐) */
    uint16_t height;   /* 垂直像素 */
    uint8_t  quality;  /* JPEG 质量 1-100，默认 60 */
} camera_config_t;

/**
 * @brief 初始化摄像头子系统。
 *
 * 此函数会:
 *   - 初始化 I2C/SCCB 总线用于传感器控制
 *   - 自动检测连接的摄像头传感器 (SC2336, OV5647 等)
 *   - 配置 MIPI-CSI 控制器 (2-lane, RAW8→RGB565)
 *   - 初始化 ISP 处理器 (RAW8→RGB565 转换)
 *   - 初始化硬件 JPEG 编码器
 *   - 在 PSRAM 中分配帧缓冲区
 *   - 启动后台采集+编码任务
 *
 * 必须在 WiFi 和 NVS 初始化之后调用。
 *
 * @param[in] cfg  摄像头配置 (传 NULL 使用默认: 800×640, Q=60)。
 * @return 成功返回 ESP_OK。
 */
esp_err_t camera_module_init(const camera_config_t *cfg);

/**
 * @brief 获取最新 JPEG 编码的摄像头帧。
 *
 * 线程安全 — 可从 HTTP 处理任务中调用。
 * 返回指向内部缓冲区的指针。数据在下一次帧采集前有效
 * (通常 20-30 ms)。
 *
 * @param[out] jpeg_buf  指向 JPEG 帧数据的指针 (只读，不可释放)。
 * @param[out] jpeg_len  JPEG 数据大小 (字节)。
 * @return 有帧可用返回 ESP_OK，尚无帧返回 ESP_ERR_NOT_FOUND。
 */
esp_err_t camera_module_get_frame(const uint8_t **jpeg_buf, size_t *jpeg_len);

/**
 * @brief 启动摄像头推流 (由 _init 自动调用)。
 * @return 成功返回 ESP_OK。
 */
esp_err_t camera_module_start(void);

/**
 * @brief 停止摄像头推流。
 * @return 成功返回 ESP_OK。
 */
esp_err_t camera_module_stop(void);

/**
 * @brief 获取当前摄像头帧率 (~1s 窗口平滑)。
 * @return 每秒帧数。
 */
uint32_t camera_module_get_fps(void);

/**
 * @brief 获取当前帧分辨率。
 * @param[out] w  帧宽度 (可为 NULL)
 * @param[out] h  帧高度 (可为 NULL)
 */
void camera_module_get_resolution(int *w, int *h);

/**
 * @brief 导出当前自动曝光状态。
 * @param[out] brightness  平滑亮度值 (0.0 - 255.0)。
 * @param[out] exp_100us   曝光时间 (100微秒单位)。
 * @param[out] gain_idx    传感器增益寄存器索引。
 */
void camera_module_get_ae_status(float *brightness, int32_t *exp_100us, int32_t *gain_idx);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MODULE_H__ */

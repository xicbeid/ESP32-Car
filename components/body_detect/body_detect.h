/*
 * 行人检测模块 — 基于 ESP-DL PicoDet 轻量行人检测
 *
 * 独立于人脸检测运行，正面/侧面/背面均可检出。
 * 检测框为橙色，与人脸检测红框区分。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PD_MAX_BOXES  8

/* ── 检测框 ── */
typedef struct {
    uint16_t x, y, w, h;
    float    score;
} pd_box_t;

/* ── 检测结果 ── */
typedef struct {
    pd_box_t boxes[PD_MAX_BOXES];
    uint8_t  count;
    uint32_t frame_id;
} pd_result_t;

/* ═══════════════════════════════════════════════════════════ *
 *  公开 API
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief 初始化行人检测 (加载 PicoDet 模型)。
 * @param frame_w  原始帧宽度
 * @param frame_h  原始帧高度
 */
esp_err_t pedestrian_detect_init(int frame_w, int frame_h);

/** 启用/禁用检测 */
void pedestrian_detect_set_enabled(bool en);
bool pedestrian_detect_is_enabled(void);

/** 获取最新检测结果 (线程安全) */
pd_result_t pedestrian_detect_get_results(void);

/**
 * @brief 提交一帧 RGB565 数据，异步检测 (非阻塞)。
 *  由 pre-JPEG 回调调用，内部帧率控制 (~3-14Hz)。
 */
void pedestrian_detect_feed_frame(const uint8_t *rgb565, int w, int h, int stride);

/**
 * @brief 在 RGB565 缓冲上画橙色检测框。
 *  由 pre-JPEG 回调调用，原地修改。
 */
void pedestrian_detect_draw_boxes(uint8_t *rgb565, int w, int h, int stride);

#ifdef __cplusplus
}
#endif

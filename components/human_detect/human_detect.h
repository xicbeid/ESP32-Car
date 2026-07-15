/*
 * 人体检测模块 — 基于 ESP-DL MSRMNP 的人脸检测
 *
 * 模型: MSR (Multi-Scale Retina) + MNP (Multi-task Neural Predictor)
 * 检测: 人脸位置 + 5 个关键点 (左右眼/鼻子/左右嘴角)
 *
 * 架构:
 *   检测任务独立于摄像头任务运行，通过共享内存获取 RGB565 帧。
 *   检测结果用于在 JPEG 编码前画框 + /status JSON 上报。
 *
 * 依赖: espressif/esp-dl v3.1.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 常量 ── */
#define HD_MAX_BOXES  8          /* 最多检测目标数 */

/* ── 检测框 ── */
typedef struct {
    uint16_t x;                  /* 左上角 X (原始分辨率坐标) */
    uint16_t y;                  /* 左上角 Y */
    uint16_t w;                  /* 宽度 */
    uint16_t h;                  /* 高度 */
    float    score;              /* 置信度 0.0-1.0 */
    /* 5 个关键点 (原始分辨率坐标), -1 = 无效 */
    int16_t  left_eye_x, left_eye_y;
    int16_t  right_eye_x, right_eye_y;
    int16_t  nose_x, nose_y;
    int16_t  left_mouth_x, left_mouth_y;
    int16_t  right_mouth_x, right_mouth_y;
} hd_box_t;

/* ── 检测结果 ── */
typedef struct {
    hd_box_t boxes[HD_MAX_BOXES];
    uint8_t  count;              /* 检测到的目标数 */
    uint32_t frame_id;           /* 帧序号 (用于判断是否过期) */
} hd_result_t;

/* ═══════════════════════════════════════════════════════════ *
 *  公开 API
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief 初始化检测模块 (加载 ESP-DL 模型)。
 * @param frame_w  原始帧宽度 (例如 1280)
 * @param frame_h  原始帧高度 (例如 720)
 * @return ESP_OK 成功。
 */
esp_err_t human_detect_init(int frame_w, int frame_h);

/**
 * @brief 启用/禁用检测。
 */
void human_detect_set_enabled(bool en);
bool human_detect_is_enabled(void);

/**
 * @brief 获取最新检测结果 (线程安全)。
 */
hd_result_t human_detect_get_results(void);

/**
 * @brief 提交一帧 RGB565 数据进行异步检测。
 *  由摄像头任务在 pre-JPEG 回调中调用。非阻塞。
 *  内部每 N 帧才真正提交一次检测 (默认 ~3Hz)。
 *
 * @param rgb565  RGB565 帧数据 (摄像头 mmap 缓冲)
 * @param w       帧宽度 (像素)
 * @param h       帧高度 (像素)
 * @param stride  行字节数 (通常 w*2)
 */
void human_detect_feed_frame(const uint8_t *rgb565, int w, int h, int stride);

/**
 * @brief 在 RGB565 缓冲上绘制检测框和关键点。
 *  由摄像头任务在 JPEG 编码前调用，原地修改。
 *
 * @param rgb565  RGB565 帧数据 (原地修改)
 * @param w       帧宽度
 * @param h       帧高度
 * @param stride  行字节数 (通常 w*2)
 */
void human_detect_draw_boxes(uint8_t *rgb565, int w, int h, int stride);

/**
 * @brief 获取内部 C++ 检测器指针 (供 face_recognition 使用)
 */
void *human_detect_get_detector(void);

/**
 * @brief 获取最新快照缓冲 (供 face_recognition 录入使用)
 * @param w  出参: 快照宽度
 * @param h  出参: 快照高度
 * @return 快照 RGB565 缓冲指针 (可能为 NULL)
 */
const uint8_t *human_detect_get_snapshot(int *w, int *h);

#ifdef __cplusplus
}
#endif

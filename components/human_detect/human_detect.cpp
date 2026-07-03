/*
 * 人体检测模块 — C 包装层 → ESP-DL MSRMNP 人脸检测
 *
 * 检测管线:
 *   feed_frame() → 下采样快照 → 检测任务 → MSR → MNP → 关键点
 *   draw_boxes() → 在原始帧上画红框 + 绿点关键点
 *
 * 模型来自 esp_brookesia_phone/components/human_face_detect/
 *   - MSR: 全图候选区域扫描
 *   - MNP: 候选区域精细检测 + 5 关键点回归
 */

/* ═══ extern "C" C API (供 uart.c / web_control.c 调用) ═══ */
extern "C" {

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "human_detect.h"

} /* extern "C" */

#include "human_face_detect.hpp"
#include <list>

#define TAG "HuDet"

/* ── 检测参数 ── */
#define DETECT_FPS         3      /* 目标检测帧率 (Hz) */
#define DETECT_CAMERA_FPS  30     /* 摄像头帧率 */
#define DETECT_SKIP_FRAMES (DETECT_CAMERA_FPS / DETECT_FPS)  /* 10 */

/* 下采样快照分辨率 (用于减少 PSRAM 拷贝量, ESP-DL 会进一步缩放) */
#define SNAP_W  640
#define SNAP_H  360
#define SNAP_BUF_SIZE  (SNAP_W * SNAP_H * 2)  /* RGB565 */

/* GUI 颜色 */
#define BOX_COLOR         0xF800  /* RGB565 纯红 */
#define BOX_THICKNESS     3       /* 线宽 */
#define KP_EYE_COLOR      0x07FF  /* 青色 */
#define KP_NOSE_COLOR     0xFFE0  /* 黄色 */
#define KP_MOUTH_COLOR    0xF81F  /* 粉色 */
#define KP_DOT_SIZE       4       /* 关键点直径 (像素) */

/* ── 内部状态 ── */
static struct {
    bool      initialized;
    bool      enabled;
    bool      busy;              /* 检测进行中, 跳过新帧 */
    int       frame_w, frame_h;  /* 原始帧分辨率 */
    float     scale_x, scale_y;  /* 快照 → 原始 缩放比 */
    float     iscale_x, iscale_y;/* 原始 → 快照 */

    uint8_t  *snapshot;          /* PSRAM 快照 RGB565 */
    int       snap_w, snap_h;

    hd_result_t result;
    uint32_t    frame_id;
    uint32_t    skip_counter;

    SemaphoreHandle_t mutex;
    SemaphoreHandle_t feed_sem;
    TaskHandle_t task;

    HumanFaceDetect *detector;   /* ESP-DL 检测器 */
} g_hd;

/* ── RGB565 下采样 (像素跳跃, 快速) ── */
static void snapshot_rgb565(const uint8_t *src, int sw, int sh, int sstride,
                            uint8_t *dst, int dw, int dh, int dstride)
{
    for (int y = 0; y < dh; y++) {
        int sy = y * sh / dh;
        const uint16_t *sline = (const uint16_t *)(src + sy * sstride);
        uint16_t *dline = (uint16_t *)(dst + y * dstride);
        for (int x = 0; x < dw; x++) {
            int sx = x * sw / dw;
            dline[x] = sline[sx];
        }
    }
}

/* ── 画矩形 (原地修改 RGB565) ── */
static void draw_rect_rgb565(uint8_t *buf, int w, int h, int stride,
                              int rx, int ry, int rw, int rh,
                              uint16_t color, int thickness)
{
    if (rx < 0) { rw += rx; rx = 0; }
    if (ry < 0) { rh += ry; ry = 0; }
    if (rx + rw > w) rw = w - rx;
    if (ry + rh > h) rh = h - ry;
    if (rw <= 0 || rh <= 0) return;

    for (int t = 0; t < thickness; t++) {
        int top = ry + t, bot = ry + rh - 1 - t;
        int left = rx + t, right = rx + rw - 1 - t;

        if (top >= 0 && top < h) {
            for (int x = left; x <= right; x++) {
                if (x >= 0 && x < w) {
                    uint8_t *p = buf + top * stride + x * 2;
                    p[0] = color & 0xFF;
                    p[1] = (color >> 8) & 0xFF;
                }
            }
        }
        if (bot >= 0 && bot < h && bot != top) {
            for (int x = left; x <= right; x++) {
                if (x >= 0 && x < w) {
                    uint8_t *p = buf + bot * stride + x * 2;
                    p[0] = color & 0xFF;
                    p[1] = (color >> 8) & 0xFF;
                }
            }
        }

        for (int y = top + 1; y < bot; y++) {
            if (y >= 0 && y < h) {
                if (left >= 0 && left < w) {
                    uint8_t *p = buf + y * stride + left * 2;
                    p[0] = color & 0xFF;
                    p[1] = (color >> 8) & 0xFF;
                }
                if (right >= 0 && right < w && right != left) {
                    uint8_t *p = buf + y * stride + right * 2;
                    p[0] = color & 0xFF;
                    p[1] = (color >> 8) & 0xFF;
                }
            }
        }
    }
}

/* ── 画实心圆点 ── */
static void draw_dot_rgb565(uint8_t *buf, int w, int h, int stride,
                             int cx, int cy, int size, uint16_t color)
{
    int r = size / 2;
    for (int dy = -r; dy <= r; dy++) {
        int y = cy + dy;
        if (y < 0 || y >= h) continue;
        for (int dx = -r; dx <= r; dx++) {
            int x = cx + dx;
            if (x < 0 || x >= w) continue;
            if (dx * dx + dy * dy <= r * r) {
                uint8_t *p = buf + y * stride + x * 2;
                p[0] = color & 0xFF;
                p[1] = (color >> 8) & 0xFF;
            }
        }
    }
}

/* ── 检测任务 ── */
static void detect_task(void *arg)
{
    ESP_LOGI(TAG, "检测任务启动 (ESP-DL MSRMNP)");

    while (1) {
        if (xSemaphoreTake(g_hd.feed_sem, pdMS_TO_TICKS(100)) != pdTRUE)
            continue;

        if (!g_hd.enabled) {
            g_hd.busy = false;
            continue;
        }
        if (!g_hd.snapshot) {
            g_hd.busy = false;
            continue;
        }

        /* ── ESP-DL 推理 ── */
        dl::image::img_t img;
        img.data = g_hd.snapshot;
        img.width = g_hd.snap_w;
        img.height = g_hd.snap_h;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;

        hd_box_t boxes[HD_MAX_BOXES];
        memset(boxes, 0, sizeof(boxes));
        int count = 0;

        auto &results = g_hd.detector->run(img);
        for (const auto &r : results) {
            if (count >= HD_MAX_BOXES) break;
            if (r.box.size() < 4) continue;
            if (r.score < 0.35f) continue;  /* 置信度过滤 (宽松, 暗光补偿) */

            hd_box_t *b = &boxes[count];
            /* 快照坐标 → 原始坐标 */
            b->x    = (uint16_t)(r.box[0] * g_hd.scale_x + 0.5f);
            b->y    = (uint16_t)(r.box[1] * g_hd.scale_y + 0.5f);
            b->w    = (uint16_t)((r.box[2] - r.box[0]) * g_hd.scale_x + 0.5f);
            b->h    = (uint16_t)((r.box[3] - r.box[1]) * g_hd.scale_y + 0.5f);
            b->score = r.score;

            /* 关键点 (快照坐标 → 原始坐标) */
            if (r.keypoint.size() >= 10) {
                b->left_eye_x    = (int16_t)(r.keypoint[0] * g_hd.scale_x + 0.5f);
                b->left_eye_y    = (int16_t)(r.keypoint[1] * g_hd.scale_y + 0.5f);
                b->right_eye_x   = (int16_t)(r.keypoint[2] * g_hd.scale_x + 0.5f);
                b->right_eye_y   = (int16_t)(r.keypoint[3] * g_hd.scale_y + 0.5f);
                b->nose_x        = (int16_t)(r.keypoint[4] * g_hd.scale_x + 0.5f);
                b->nose_y        = (int16_t)(r.keypoint[5] * g_hd.scale_y + 0.5f);
                b->left_mouth_x  = (int16_t)(r.keypoint[6] * g_hd.scale_x + 0.5f);
                b->left_mouth_y  = (int16_t)(r.keypoint[7] * g_hd.scale_y + 0.5f);
                b->right_mouth_x = (int16_t)(r.keypoint[8] * g_hd.scale_x + 0.5f);
                b->right_mouth_y = (int16_t)(r.keypoint[9] * g_hd.scale_y + 0.5f);
            } else {
                memset(&b->left_eye_x, -1, 10 * sizeof(int16_t));
            }

            count++;
        }

        /* 发布结果 */
        if (xSemaphoreTake(g_hd.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memset(&g_hd.result, 0, sizeof(g_hd.result));
            g_hd.result.count = (uint8_t)count;
            g_hd.result.frame_id = g_hd.frame_id;
            memcpy(g_hd.result.boxes, boxes, count * sizeof(hd_box_t));
            xSemaphoreGive(g_hd.mutex);
        }

        if (count > 0) {
            ESP_LOGI(TAG, "检测到 %d 个人脸", count);
        }

        g_hd.busy = false;
    }
}

/* ═══════════════════════════════════════════════════════════ *
 *  公开 API (extern "C")
 * ═══════════════════════════════════════════════════════════ */

extern "C" {

esp_err_t human_detect_init(int frame_w, int frame_h)
{
    if (g_hd.initialized) return ESP_OK;

    memset(&g_hd, 0, sizeof(g_hd));
    g_hd.frame_w = frame_w;
    g_hd.frame_h = frame_h;
    g_hd.snap_w  = SNAP_W;
    g_hd.snap_h  = SNAP_H;
    g_hd.scale_x = (float)frame_w / (float)SNAP_W;
    g_hd.scale_y = (float)frame_h / (float)SNAP_H;
    g_hd.iscale_x = (float)SNAP_W / (float)frame_w;
    g_hd.iscale_y = (float)SNAP_H / (float)frame_h;
    g_hd.enabled = false;

    /* 分配快照缓冲区 (PSRAM, ~460KB) */
    g_hd.snapshot = (uint8_t *)heap_caps_malloc(SNAP_BUF_SIZE,
                            MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!g_hd.snapshot) {
        ESP_LOGE(TAG, "快照 PSRAM 分配失败 (%d 字节)", SNAP_BUF_SIZE);
        return ESP_ERR_NO_MEM;
    }

    /* 创建信号量 */
    g_hd.mutex    = xSemaphoreCreateMutex();
    g_hd.feed_sem = xSemaphoreCreateBinary();
    if (!g_hd.mutex || !g_hd.feed_sem) {
        ESP_LOGE(TAG, "信号量创建失败");
        return ESP_ERR_NO_MEM;
    }

    /* 创建检测任务 (大栈: ESP-DL 推理递归深度) */
    BaseType_t ret = xTaskCreate(detect_task, "HuDetTask",
                                  12288, NULL, tskIDLE_PRIORITY + 3,
                                  &g_hd.task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "检测任务创建失败");
        return ESP_ERR_NO_MEM;
    }

    /* 初始化 ESP-DL 模型 */
    g_hd.detector = new HumanFaceDetect();
    if (!g_hd.detector) {
        ESP_LOGE(TAG, "ESP-DL 模型初始化失败");
        return ESP_ERR_NO_MEM;
    }

    g_hd.initialized = true;
    ESP_LOGI(TAG, "ESP-DL 人脸检测就绪 (%dx%d → %dx%d 快照)",
             frame_w, frame_h, SNAP_W, SNAP_H);
    return ESP_OK;
}

void human_detect_set_enabled(bool en)
{
    g_hd.enabled = en;
    if (en) {
        g_hd.skip_counter = 0;
        ESP_LOGI(TAG, "人脸检测已启用");
    } else {
        ESP_LOGI(TAG, "人脸检测已禁用");
    }
}

bool human_detect_is_enabled(void)
{
    return g_hd.enabled;
}

hd_result_t human_detect_get_results(void)
{
    hd_result_t r;
    memset(&r, 0, sizeof(r));
    if (xSemaphoreTake(g_hd.mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        r = g_hd.result;
        xSemaphoreGive(g_hd.mutex);
    }
    return r;
}

void human_detect_feed_frame(const uint8_t *rgb565, int w, int h, int stride)
{
    if (!g_hd.enabled || !g_hd.initialized) return;
    if (!rgb565) return;
    if (g_hd.busy) return;  /* 上一帧检测未完成, 跳过 */

    /* 帧率控制: 每 ~10 帧运行一次 (30fps → 3Hz) */
    g_hd.skip_counter++;
    if (g_hd.skip_counter < DETECT_SKIP_FRAMES) return;
    g_hd.skip_counter = 0;

    g_hd.busy = true;
    g_hd.frame_id++;

    /* 下采样拷贝到快照缓冲区 (PSRAM, 约 1ms) */
    snapshot_rgb565(rgb565, w, h, stride,
                    g_hd.snapshot, SNAP_W, SNAP_H, SNAP_W * 2);

    /* 通知检测任务 */
    xSemaphoreGive(g_hd.feed_sem);
}

void human_detect_draw_boxes(uint8_t *rgb565, int w, int h, int stride)
{
    if (!g_hd.enabled || !g_hd.initialized) return;
    if (!rgb565) return;

    hd_result_t r = human_detect_get_results();
    /* 只画最近 30 帧内的结果 (~1s) */
    if (g_hd.frame_id - r.frame_id > 30) return;

    for (int i = 0; i < r.count; i++) {
        hd_box_t *b = &r.boxes[i];

        /* 红框 */
        draw_rect_rgb565(rgb565, w, h, stride,
                          b->x, b->y, b->w, b->h,
                          BOX_COLOR, BOX_THICKNESS);

        /* 关键点 (跳过无效 -1) */
        if (b->left_eye_x > 0)
            draw_dot_rgb565(rgb565, w, h, stride,
                            b->left_eye_x, b->left_eye_y,
                            KP_DOT_SIZE, KP_EYE_COLOR);
        if (b->right_eye_x > 0)
            draw_dot_rgb565(rgb565, w, h, stride,
                            b->right_eye_x, b->right_eye_y,
                            KP_DOT_SIZE, KP_EYE_COLOR);
        if (b->nose_x > 0)
            draw_dot_rgb565(rgb565, w, h, stride,
                            b->nose_x, b->nose_y,
                            KP_DOT_SIZE, KP_NOSE_COLOR);
        if (b->left_mouth_x > 0)
            draw_dot_rgb565(rgb565, w, h, stride,
                            b->left_mouth_x, b->left_mouth_y,
                            KP_DOT_SIZE, KP_MOUTH_COLOR);
        if (b->right_mouth_x > 0)
            draw_dot_rgb565(rgb565, w, h, stride,
                            b->right_mouth_x, b->right_mouth_y,
                            KP_DOT_SIZE, KP_MOUTH_COLOR);
    }
}

} /* extern "C" */

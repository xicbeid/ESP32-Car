/*
 * 行人检测模块 — C 包装层 → ESP-DL PicoDet
 *
 * 管线: feed_frame() → 下采样快照 → 信号量 → PedDetTask
 *       → PicoDet → 坐标映射 → 发布结果
 *       draw_boxes() → 橙色框 (0xFD20)
 */

extern "C" {

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "body_detect.h"

} /* extern "C" */

#include "pedestrian_detect.hpp"
#include <list>

#define TAG "PedDet"

#define DETECT_FPS         7      /* 目标检测帧率 (Hz), 20fps÷3≈6.7Hz */
#define DETECT_CAMERA_FPS  20     /* 实际 ISP JPEG 输出帧率 */
#define DETECT_SKIP_FRAMES (DETECT_CAMERA_FPS / DETECT_FPS)  /* 每3帧检测1次 */

#define SNAP_W  640
#define SNAP_H  360
#define SNAP_BUF_SIZE  (SNAP_W * SNAP_H * 2)

#define PED_BOX_COLOR     0xFD20
#define PED_BOX_THICKNESS 2

static struct {
    bool      initialized;
    bool      enabled;
    bool      busy;
    int       frame_w, frame_h;
    float     scale_x, scale_y;
    uint8_t  *snapshot;
    int       snap_w, snap_h;
    pd_result_t result;
    uint32_t    frame_id;
    uint32_t    skip_counter;
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t feed_sem;
    TaskHandle_t task;
    void      *detector;
} g_pd;

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
/* ── 暗光增强：当画面偏暗时做伽马提亮 ── */
static void snapshot_enhance_rgb565(uint8_t *buf, int w, int h, int stride)
{
    uint8_t max_g = 0;
    for (int y = 0; y < h; y++) {
        const uint16_t *line = (const uint16_t *)(buf + y * stride);
        for (int x = 0; x < w; x++) {
            uint8_t g6 = (line[x] >> 5) & 0x3F;
            uint8_t g8 = (g6 << 2) | (g6 >> 4);
            if (g8 > max_g) max_g = g8;
        }
    }
    if (max_g >= 100) return;

    float scale = 1.0f + (100.0f - max_g) / 80.0f;
    uint8_t g_lut[64];
    for (int i = 0; i < 64; i++) {
        float v = sqrtf((float)i / 63.0f) * 63.0f * scale;
        if (v > 63.0f) v = 63.0f;
        g_lut[i] = (uint8_t)(v + 0.5f);
    }

    for (int y = 0; y < h; y++) {
        uint16_t *line = (uint16_t *)(buf + y * stride);
        for (int x = 0; x < w; x++) {
            uint16_t p = line[x];
            uint8_t r5 = (p >> 11) & 0x1F;
            uint8_t g6 = (p >> 5) & 0x3F;
            uint8_t b5 = p & 0x1F;
            uint8_t new_g6 = g_lut[g6];
            float ratio = (g6 > 0) ? (float)new_g6 / (float)g6 : scale;
            int new_r5 = (int)(r5 * ratio + 0.5f);
            int new_b5 = (int)(b5 * ratio + 0.5f);
            if (new_r5 > 31) new_r5 = 31;
            if (new_b5 > 31) new_b5 = 31;
            line[x] = ((uint16_t)new_r5 << 11) | ((uint16_t)new_g6 << 5) | (uint16_t)new_b5;
        }
    }
}

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
            for (int x = left; x <= right && x < w; x++) {
                uint8_t *p = buf + top * stride + x * 2;
                p[0] = color & 0xFF; p[1] = (color >> 8) & 0xFF;
            }
        }
        if (bot >= 0 && bot < h && bot != top) {
            for (int x = left; x <= right && x < w; x++) {
                uint8_t *p = buf + bot * stride + x * 2;
                p[0] = color & 0xFF; p[1] = (color >> 8) & 0xFF;
            }
        }
        for (int y = top + 1; y < bot; y++) {
            if (y >= 0 && y < h) {
                if (left >= 0 && left < w) {
                    uint8_t *p = buf + y * stride + left * 2;
                    p[0] = color & 0xFF; p[1] = (color >> 8) & 0xFF;
                }
                if (right >= 0 && right < w && right != left) {
                    uint8_t *p = buf + y * stride + right * 2;
                    p[0] = color & 0xFF; p[1] = (color >> 8) & 0xFF;
                }
            }
        }
    }
}

static void ped_detect_task(void *arg)
{
    ESP_LOGI(TAG, "PedDet task running (PicoDet)");
    while (1) {
        if (xSemaphoreTake(g_pd.feed_sem, pdMS_TO_TICKS(100)) != pdTRUE)
            continue;
        if (!g_pd.enabled || !g_pd.snapshot) {
            g_pd.busy = false; continue;
        }
        dl::image::img_t img;
        img.data = g_pd.snapshot;
        img.width = g_pd.snap_w; img.height = g_pd.snap_h;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;

        pd_box_t boxes[PD_MAX_BOXES];
        memset(boxes, 0, sizeof(boxes));
        int count = 0;

        PedestrianDetect *det = (PedestrianDetect *)g_pd.detector;
        auto &results = det->run(img);
        for (const auto &r : results) {
            if (count >= PD_MAX_BOXES) break;
            if (r.box.size() < 4) continue;
            if (r.score < 0.35f) continue;  /* 暗光场景放宽 */
            pd_box_t *b = &boxes[count];
            b->x = (uint16_t)(r.box[0] * g_pd.scale_x + 0.5f);
            b->y = (uint16_t)(r.box[1] * g_pd.scale_y + 0.5f);
            b->w = (uint16_t)((r.box[2] - r.box[0]) * g_pd.scale_x + 0.5f);
            b->h = (uint16_t)((r.box[3] - r.box[1]) * g_pd.scale_y + 0.5f);
            b->score = r.score;
            count++;
        }
        if (xSemaphoreTake(g_pd.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memset(&g_pd.result, 0, sizeof(g_pd.result));
            g_pd.result.count = (uint8_t)count;
            g_pd.result.frame_id = g_pd.frame_id;
            memcpy(g_pd.result.boxes, boxes, count * sizeof(pd_box_t));
            xSemaphoreGive(g_pd.mutex);
        }
        if (count > 0) ESP_LOGI(TAG, "Detected %d persons", count);
        g_pd.busy = false;
    }
}

extern "C" {

esp_err_t pedestrian_detect_init(int frame_w, int frame_h)
{
    if (g_pd.initialized) return ESP_OK;
    memset(&g_pd, 0, sizeof(g_pd));
    g_pd.frame_w = frame_w; g_pd.frame_h = frame_h;
    g_pd.snap_w = SNAP_W; g_pd.snap_h = SNAP_H;
    g_pd.scale_x = (float)frame_w / (float)SNAP_W;
    g_pd.scale_y = (float)frame_h / (float)SNAP_H;
    g_pd.enabled = false;

    g_pd.snapshot = (uint8_t *)heap_caps_malloc(SNAP_BUF_SIZE,
                            MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!g_pd.snapshot) {
        ESP_LOGE(TAG, "Snapshot PSRAM alloc failed"); return ESP_ERR_NO_MEM;
    }
    g_pd.mutex = xSemaphoreCreateMutex();
    g_pd.feed_sem = xSemaphoreCreateBinary();
    if (!g_pd.mutex || !g_pd.feed_sem) {
        ESP_LOGE(TAG, "Semaphore create failed"); return ESP_ERR_NO_MEM;
    }
    g_pd.detector = new PedestrianDetect();
    if (!g_pd.detector) {
        ESP_LOGE(TAG, "PicoDet model load failed"); return ESP_ERR_NO_MEM;
    }
    BaseType_t ret = xTaskCreate(ped_detect_task, "PedDetTask",
                                  12288, NULL, tskIDLE_PRIORITY + 3, &g_pd.task);
    if (ret != pdPASS) { ESP_LOGE(TAG, "Task create failed"); return ESP_ERR_NO_MEM; }
    g_pd.initialized = true;
    ESP_LOGI(TAG, "PicoDet ready (%dx%d -> %dx%d)", frame_w, frame_h, SNAP_W, SNAP_H);
    return ESP_OK;
}

void pedestrian_detect_set_enabled(bool en) {
    g_pd.enabled = en;
    if (en) { g_pd.skip_counter = 0; ESP_LOGI(TAG, "Enabled"); }
    else ESP_LOGI(TAG, "Disabled");
}
bool pedestrian_detect_is_enabled(void) { return g_pd.enabled; }

pd_result_t pedestrian_detect_get_results(void) {
    pd_result_t r; memset(&r, 0, sizeof(r));
    if (xSemaphoreTake(g_pd.mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        r = g_pd.result; xSemaphoreGive(g_pd.mutex);
    }
    return r;
}

void pedestrian_detect_feed_frame(const uint8_t *rgb565, int w, int h, int stride) {
    if (!g_pd.enabled || !g_pd.initialized || !rgb565 || g_pd.busy) return;
    g_pd.skip_counter++;
    if (g_pd.skip_counter < DETECT_SKIP_FRAMES) return;
    g_pd.skip_counter = 0;
    g_pd.busy = true; g_pd.frame_id++;
    snapshot_rgb565(rgb565, w, h, stride, g_pd.snapshot, SNAP_W, SNAP_H, SNAP_W * 2);
    snapshot_enhance_rgb565(g_pd.snapshot, SNAP_W, SNAP_H, SNAP_W * 2);
    esp_cache_msync(g_pd.snapshot, SNAP_BUF_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    xSemaphoreGive(g_pd.feed_sem);
}

void pedestrian_detect_draw_boxes(uint8_t *rgb565, int w, int h, int stride) {
    if (!g_pd.enabled || !g_pd.initialized || !rgb565) return;

    /* 缓存结果 — 只在新数据到来时更新，持续画同一组框避免闪烁 */
    static pd_result_t cached;
    static uint32_t    cached_frame_id = 0;

    pd_result_t r = pedestrian_detect_get_results();

    if (r.count > 0 && r.frame_id > 0 &&
        g_pd.frame_id - r.frame_id <= 30 &&
        r.frame_id != cached_frame_id) {
        cached = r;
        cached_frame_id = r.frame_id;
    }

    if (cached_frame_id == 0) return;

    if (g_pd.frame_id - cached_frame_id > 30) {
        cached_frame_id = 0;
        return;
    }

    for (int i = 0; i < cached.count; i++) {
        pd_box_t *b = &cached.boxes[i];
        draw_rect_rgb565(rgb565, w, h, stride, b->x, b->y, b->w, b->h,
                          PED_BOX_COLOR, PED_BOX_THICKNESS);
    }
}

} /* extern "C" */

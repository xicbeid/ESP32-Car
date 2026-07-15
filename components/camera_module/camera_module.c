/*
 * Camera module — V4L2 MIPI-CSI MMAP + ISP + HW JPEG
 *
 * SC2336 RAW10 640x480@50fps → MIPI-CSI → ISP (RGB565, AE/AWB/Gamma auto)
 * → DMA copy buffer (non-cacheable PSRAM) → draw overlay → HW JPEG → MJPEG
 *
 * Key design: DMA buffer (MALLOC_CAP_DMA) avoids all CPU cache sync.
 * Official esp_brookesia_phone uses LVGL (CPU reads — transparent through cache).
 * We use JPEG encode (DMA reads — must go through non-cacheable memory).
 */
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "driver/jpeg_encode.h"
#include "camera_module.h"

#define TAG "Camera"

#define CAM_DEFAULT_QUALITY  55
#define CAM_TASK_STACK       8192
#define CAM_TASK_PRIO        5
#define JPEG_BUF_SIZE        (256 * 1024)
#define MAX_FB_NUM           3

static int               s_video_fd = -1;
static jpeg_encoder_handle_t s_jpeg_enc = NULL;
static jpeg_encode_cfg_t s_jpeg_cfg;
static uint8_t          *s_jpeg_enc_buf = NULL;
static size_t            s_jpeg_enc_buf_size = 0;

static uint8_t  *s_mmap_buf[MAX_FB_NUM];
static uint8_t  *s_dma_buf = NULL;    /* non-cacheable PSRAM for JPEG input */
static uint32_t  s_fb_stride  = 0;
static uint32_t  s_fb_width   = 0;
static uint32_t  s_fb_height  = 0;
static int       s_nbufs = 0;

static TaskHandle_t      s_cam_task = NULL;
static SemaphoreHandle_t s_frame_mutex = NULL;
static SemaphoreHandle_t s_frame_ready = NULL;

static camera_pre_jpeg_cb_t s_pre_jpeg_cb = NULL;
static void                *s_pre_jpeg_ctx = NULL;

static uint8_t  *s_jpeg_out[2] = {NULL, NULL};
static uint32_t  s_jpeg_out_len[2] = {0, 0};
static volatile int s_published_idx = 0;
static volatile bool s_has_frame = false;

static volatile uint32_t s_fps_counter = 0;
static int64_t           s_fps_last_us = 0;
static volatile uint32_t s_cached_fps = 0;

static uint8_t *cam_alloc(size_t size)
{
    uint8_t *p = heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!p) p = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    return p;
}

static void camera_task(void *arg)
{
    uint32_t fc = 0;
    int write_idx = 1;
    uint32_t buf_sz = s_fb_stride * s_fb_height;
    ESP_LOGI(TAG, "Cam task: %" PRIu32 "x%" PRIu32 " stride=%" PRIu32,
             s_fb_width, s_fb_height, s_fb_stride);

    while (1) {
        struct v4l2_buffer vbuf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP,
        };

        if (ioctl(s_video_fd, VIDIOC_DQBUF, &vbuf) != 0) {
            vTaskDelay(pdMS_TO_TICKS(10)); continue;
        }

        if (!(vbuf.flags & V4L2_BUF_FLAG_DONE) ||
            (vbuf.flags & V4L2_BUF_FLAG_ERROR)) {
            ioctl(s_video_fd, VIDIOC_QBUF, &vbuf);
            continue;
        }

        uint8_t *src = s_mmap_buf[vbuf.index];
        if (!src) { ioctl(s_video_fd, VIDIOC_QBUF, &vbuf); continue; }

        /* Copy frame to non-cacheable DMA buffer.
         * CPU reads from MMAP buf (V4L2 driver handles cache → fresh data).
         * CPU writes to DMA buf → goes directly to PSRAM (no cache). */
        memcpy(s_dma_buf, src, buf_sz);

        /* Draw detection boxes on DMA buffer — writes go straight to PSRAM.
         * No cache sync needed — JPEG DMA will see the pixels immediately. */
        if (s_pre_jpeg_cb) {
            s_pre_jpeg_cb(s_dma_buf, (int)s_fb_width, (int)s_fb_height,
                          (int)s_fb_stride, s_pre_jpeg_ctx);
        }

        /* JPEG encode from DMA buffer.
         * JPEG HW reads from non-cacheable PSRAM via DMA → always sees fresh data. */
        uint32_t out_len = 0;
        esp_err_t ret = jpeg_encoder_process(
            s_jpeg_enc, &s_jpeg_cfg, s_dma_buf, buf_sz,
            s_jpeg_enc_buf, (uint32_t)s_jpeg_enc_buf_size, &out_len);

        if (ret == ESP_OK && out_len > 0 && out_len <= JPEG_BUF_SIZE) {
            s_fps_counter++;

            /* JPEG output buffer is cached PSRAM (jpeg_alloc_encoder_mem).
             * M2C: JPEG HW wrote via DMA → CPU must reload from RAM. */
            uint32_t aligned = (out_len + 63) & ~63U;
            esp_cache_msync(s_jpeg_enc_buf, aligned, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

            if (!s_jpeg_out[write_idx])
                s_jpeg_out[write_idx] = cam_alloc(JPEG_BUF_SIZE);

            if (s_jpeg_out[write_idx]) {
                if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    memcpy(s_jpeg_out[write_idx], s_jpeg_enc_buf, out_len);
                    s_jpeg_out_len[write_idx] = out_len;
                    s_published_idx = write_idx;
                    s_has_frame = true;
                    write_idx = 1 - write_idx;
                    xSemaphoreGive(s_frame_mutex);
                    xSemaphoreTake(s_frame_ready, 0);
                    xSemaphoreGive(s_frame_ready);
                }
            }
            if (++fc % 100 == 0)
                ESP_LOGI(TAG, "Frame#%" PRIu32 ": %" PRIu32 "B", fc, out_len);
        }

        ioctl(s_video_fd, VIDIOC_QBUF, &vbuf);
    }
}

/* ═══ Public API ═══ */

esp_err_t camera_module_init(const camera_config_t *cfg)
{
    uint8_t quality = cfg ? cfg->quality : CAM_DEFAULT_QUALITY;

    ESP_LOGI(TAG, "[1/4] esp_video_init...");
    esp_video_init_csi_config_t csi_cfg = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = { .port = 0, .scl_pin = 8, .sda_pin = 7 }, .freq = 100000,
        },
        .reset_pin = -1, .pwdn_pin = -1,
    };
    esp_video_init_config_t vcfg = { .csi = &csi_cfg };
    ESP_RETURN_ON_ERROR(esp_video_init(&vcfg), TAG, "esp_video_init");

    ESP_LOGI(TAG, "[2/4] Open /dev/video0 → ISP→RGB565...");
    s_video_fd = open("/dev/video0", O_RDWR);
    if (s_video_fd < 0) { ESP_LOGE(TAG, "open failed"); goto fail0; }

    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    if (ioctl(s_video_fd, VIDIOC_G_FMT, &fmt) != 0) { ESP_LOGE(TAG, "G_FMT"); goto fail0; }
    s_fb_width  = (uint32_t)fmt.fmt.pix.width;
    s_fb_height = (uint32_t)fmt.fmt.pix.height;
    s_fb_stride = fmt.fmt.pix.bytesperline ? (uint32_t)fmt.fmt.pix.bytesperline : s_fb_width * 2;
    if (s_fb_stride < s_fb_width * 2) s_fb_stride = s_fb_width * 2;
    ESP_LOGI(TAG, "Sensor: %" PRIu32 "x%" PRIu32 " stride=%" PRIu32,
             s_fb_width, s_fb_height, s_fb_stride);

    /* DMA buffer: non-cacheable PSRAM for JPEG input + box overlay */
    uint32_t buf_sz = s_fb_stride * s_fb_height;
    s_dma_buf = heap_caps_calloc(1, buf_sz, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (!s_dma_buf) { ESP_LOGE(TAG, "DMA buf alloc fail (%u bytes)", (unsigned)buf_sz); goto fail0; }
    ESP_LOGI(TAG, "DMA buf: %u bytes at %p (non-cacheable)", (unsigned)buf_sz, s_dma_buf);

    ESP_LOGI(TAG, "[3/4] MMAP buffers ×%d...", MAX_FB_NUM);
    struct v4l2_requestbuffers req = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .count = MAX_FB_NUM,
    };
    if (ioctl(s_video_fd, VIDIOC_REQBUFS, &req) != 0) { ESP_LOGE(TAG, "REQBUFS"); goto fail0; }
    s_nbufs = (int)req.count;

    for (int i = 0; i < s_nbufs; i++) {
        struct v4l2_buffer q = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = (uint32_t)i };
        if (ioctl(s_video_fd, VIDIOC_QUERYBUF, &q) != 0) { ESP_LOGE(TAG, "QUERYBUF[%d]", i); goto fail0; }
        s_mmap_buf[i] = mmap(NULL, q.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_video_fd, q.m.offset);
        if (s_mmap_buf[i] == MAP_FAILED) { ESP_LOGE(TAG, "mmap[%d]", i); goto fail0; }
    }
    for (int i = 0; i < s_nbufs; i++) {
        struct v4l2_buffer q = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = (uint32_t)i };
        if (ioctl(s_video_fd, VIDIOC_QBUF, &q) != 0) { ESP_LOGE(TAG, "QBUF[%d]", i); goto fail0; }
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(s_video_fd, VIDIOC_STREAMON, &type) != 0) { ESP_LOGE(TAG, "STREAMON"); goto fail0; }

    ESP_LOGI(TAG, "[4/4] JPEG Q=%d...", quality);
    jpeg_encode_engine_cfg_t eng_cfg = { .timeout_ms = 5000 };
    if (jpeg_new_encoder_engine(&eng_cfg, &s_jpeg_enc) != ESP_OK) { ESP_LOGE(TAG, "JPEG eng"); goto fail0; }
    jpeg_encode_memory_alloc_cfg_t mem_cfg = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    size_t allocated = 0;
    s_jpeg_enc_buf = (uint8_t *)jpeg_alloc_encoder_mem(JPEG_BUF_SIZE, &mem_cfg, &allocated);
    if (!s_jpeg_enc_buf) { ESP_LOGE(TAG, "JPEG buf"); goto fail0; }
    s_jpeg_enc_buf_size = allocated;
    s_jpeg_cfg.height = s_fb_height;
    s_jpeg_cfg.width = s_fb_width;
    s_jpeg_cfg.image_quality = quality;
    s_jpeg_cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB565;
    s_jpeg_cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV422;

    s_jpeg_out[0] = cam_alloc(JPEG_BUF_SIZE);
    s_jpeg_out[1] = cam_alloc(JPEG_BUF_SIZE);
    s_frame_mutex = xSemaphoreCreateMutex();
    s_frame_ready = xSemaphoreCreateBinary();

    if (xTaskCreate(camera_task, "CamV4L2", CAM_TASK_STACK, NULL, CAM_TASK_PRIO, &s_cam_task) != pdPASS) {
        ESP_LOGE(TAG, "task"); goto fail0;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Camera ready! %" PRIu32 "x%" PRIu32 " ISP AE/AWB/Gamma=auto DMA-overlay",
             s_fb_width, s_fb_height);
    return ESP_OK;

fail0:
    if (s_video_fd >= 0) { close(s_video_fd); s_video_fd = -1; }
    esp_video_deinit();
    return ESP_FAIL;
}

esp_err_t camera_module_get_frame(const uint8_t **jpeg_buf, size_t *jpeg_len)
{
    if (!jpeg_buf || !jpeg_len) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(200)) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;
    int idx = s_published_idx;
    *jpeg_buf = s_jpeg_out[idx];
    *jpeg_len = s_jpeg_out_len[idx];
    xSemaphoreGive(s_frame_mutex);
    return (*jpeg_buf && *jpeg_len) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t camera_module_start(void) { return ESP_OK; }

void camera_module_get_resolution(int *w, int *h)
{
    if (w) *w = (int)s_fb_width;
    if (h) *h = (int)s_fb_height;
}

void camera_module_set_pre_jpeg_cb(camera_pre_jpeg_cb_t cb, void *user_ctx)
{
    s_pre_jpeg_cb = cb;
    s_pre_jpeg_ctx = user_ctx;
}

uint32_t camera_module_get_fps(void)
{
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - s_fps_last_us;
    if (elapsed_us > 500000) {
        uint32_t count = s_fps_counter;
        s_fps_counter = 0;
        if (elapsed_us > 0)
            s_cached_fps = (uint32_t)((count * 1000000ULL) / (uint64_t)elapsed_us);
        s_fps_last_us = now_us;
    }
    return s_cached_fps;
}

void camera_module_get_ae_status(float *brightness, int32_t *exp_100us, int32_t *gain_idx)
{
    if (brightness) *brightness = 0.0f;
    if (exp_100us)  *exp_100us  = 0;
    if (gain_idx)   *gain_idx   = 0;
}

esp_err_t camera_module_stop(void)
{
    if (s_video_fd >= 0) {
        int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(s_video_fd, VIDIOC_STREAMOFF, &t);
        for (int i = 0; i < s_nbufs; i++)
            if (s_mmap_buf[i] && s_mmap_buf[i] != MAP_FAILED) munmap(s_mmap_buf[i], s_fb_stride * s_fb_height);
        close(s_video_fd); s_video_fd = -1;
    }
    if (s_jpeg_enc) { jpeg_del_encoder_engine(s_jpeg_enc); s_jpeg_enc = NULL; }
    if (s_jpeg_enc_buf) { free(s_jpeg_enc_buf); s_jpeg_enc_buf = NULL; }
    for (int i = 0; i < 2; i++) { if (s_jpeg_out[i]) { free(s_jpeg_out[i]); s_jpeg_out[i] = NULL; } }
    if (s_dma_buf) { heap_caps_free(s_dma_buf); s_dma_buf = NULL; }
    if (s_frame_mutex) { vSemaphoreDelete(s_frame_mutex); s_frame_mutex = NULL; }
    if (s_frame_ready) { vSemaphoreDelete(s_frame_ready); s_frame_ready = NULL; }
    if (s_cam_task) { vTaskDelete(s_cam_task); s_cam_task = NULL; }
    esp_video_deinit();
    ESP_LOGI(TAG, "Camera stopped");
    return ESP_OK;
}

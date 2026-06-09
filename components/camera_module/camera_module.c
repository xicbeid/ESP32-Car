/*
 * Camera Module — V4L2 MIPI-CSI camera + HW JPEG encode
 *
 * Pipeline: SC2336 → MIPI-CSI → /dev/video0 (RAW Bayer or grayscale)
 * ISP pipeline controller disabled due to IPA AWB crash.
 * Fallback: encode RAW as grayscale JPEG (usable monochrome preview).
 */

#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_check.h"
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

/* ==================== State ==================== */
static int               s_video_fd = -1;
static jpeg_encoder_handle_t s_jpeg_enc = NULL;
static jpeg_encode_cfg_t s_jpeg_cfg;
static uint8_t          *s_jpeg_buf = NULL;
static size_t            s_jpeg_buf_size = 0;

static uint8_t *s_mmap_buf[MAX_FB_NUM];
static uint32_t s_fb_size = 0;
static int      s_nbufs = 0;
static uint16_t s_sensor_w = 0, s_sensor_h = 0;
static uint32_t s_pixelformat = 0;  /* V4L2 FOURCC */

static TaskHandle_t      s_cam_task = NULL;
static SemaphoreHandle_t s_frame_mutex = NULL;

static uint8_t  *s_jpeg_out = NULL;
static uint32_t  s_jpeg_out_len = 0;
static bool      s_jpeg_ready = false;

/* ==================== JPEG Init ==================== */
static esp_err_t jpeg_init(uint8_t quality)
{
    jpeg_encode_engine_cfg_t eng_cfg = { .timeout_ms = 5000 };
    ESP_RETURN_ON_ERROR(jpeg_new_encoder_engine(&eng_cfg, &s_jpeg_enc), TAG, "JPEG engine");

    jpeg_encode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    size_t allocated = 0;
    s_jpeg_buf = (uint8_t *)jpeg_alloc_encoder_mem(JPEG_BUF_SIZE, &mem_cfg, &allocated);
    if (!s_jpeg_buf) { ESP_LOGE(TAG, "JPEG buf fail"); return ESP_ERR_NO_MEM; }
    s_jpeg_buf_size = allocated;

    /* If driver gave us RAW Bayer, encode as grayscale. If RGB565, encode as RGB565. */
    bool is_raw = (s_pixelformat == V4L2_PIX_FMT_SBGGR8 ||
                   s_pixelformat == V4L2_PIX_FMT_SGBRG8 ||
                   s_pixelformat == V4L2_PIX_FMT_SGRBG8 ||
                   s_pixelformat == V4L2_PIX_FMT_SRGGB8 ||
                   s_pixelformat == V4L2_PIX_FMT_SBGGR10 ||
                   s_pixelformat == V4L2_PIX_FMT_SGBRG10 ||
                   s_pixelformat == V4L2_PIX_FMT_SGRBG10 ||
                   s_pixelformat == V4L2_PIX_FMT_SRGGB10);

    s_jpeg_cfg.height        = s_sensor_h;
    s_jpeg_cfg.width         = s_sensor_w;
    s_jpeg_cfg.image_quality = quality;

    if (is_raw) {
        s_jpeg_cfg.src_type   = JPEG_ENCODE_IN_FORMAT_GRAY;
        s_jpeg_cfg.sub_sample = JPEG_DOWN_SAMPLING_GRAY;
        ESP_LOGW(TAG, "JPEG: RAW Bayer → grayscale (no ISP)");
    } else {
        s_jpeg_cfg.src_type   = JPEG_ENCODE_IN_FORMAT_RGB565;
        s_jpeg_cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV422;
    }
    ESP_LOGI(TAG, "JPEG: %dx%d Q=%d fmt=0x%08lx (%s)",
             s_sensor_w, s_sensor_h, quality, (unsigned long)s_pixelformat,
             is_raw ? "RAW→GRAY" : "RGB565");
    return ESP_OK;
}

/* ==================== Camera Task ==================== */
static void camera_task(void *arg)
{
    uint32_t fc = 0;
    ESP_LOGI(TAG, "Cam task running");

    while (1) {
        struct v4l2_buffer vbuf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(s_video_fd, VIDIOC_DQBUF, &vbuf) != 0) {
            vTaskDelay(pdMS_TO_TICKS(10)); continue;
        }

        uint8_t *src = s_mmap_buf[vbuf.index];
        if (!src) goto qbuf;

        uint32_t out_len = 0;
        esp_err_t ret = jpeg_encoder_process(
            s_jpeg_enc, &s_jpeg_cfg,
            src, s_fb_size, s_jpeg_buf, (uint32_t)s_jpeg_buf_size, &out_len);

        if (ret == ESP_OK && out_len > 0 && out_len <= JPEG_BUF_SIZE) {
            if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (!s_jpeg_out) s_jpeg_out = heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
                if (!s_jpeg_out) s_jpeg_out = heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_8BIT);
                if (s_jpeg_out) {
                    memcpy(s_jpeg_out, s_jpeg_buf, out_len);
                    s_jpeg_out_len = out_len;
                    s_jpeg_ready = true;
                }
                xSemaphoreGive(s_frame_mutex);
            }
            if (++fc % 50 == 0) ESP_LOGI(TAG, "F#%lu: %luB", (unsigned long)fc, (unsigned long)out_len);
        }

    qbuf:
        ioctl(s_video_fd, VIDIOC_QBUF, &vbuf);
    }
}

/* ==================== Public API ==================== */
esp_err_t camera_module_init(const camera_config_t *cfg)
{
    uint8_t quality = cfg ? cfg->quality : CAM_DEFAULT_QUALITY;

    ESP_LOGI(TAG, "  [1/3] esp_video_init...");
    esp_video_init_csi_config_t csi_cfg = {
        .sccb_config = { .init_sccb = true,
            .i2c_config = { .port = 0, .scl_pin = 8, .sda_pin = 7 }, .freq = 100000 },
        .reset_pin = -1, .pwdn_pin = -1,
    };
    esp_video_init_config_t vcfg = { .csi = &csi_cfg };
    ESP_RETURN_ON_ERROR(esp_video_init(&vcfg), TAG, "esp_video_init fail");

    ESP_LOGI(TAG, "  [2/3] V4L2 setup...");
    s_video_fd = open("/dev/video0", O_RDWR);
    if (s_video_fd < 0) { ESP_LOGE(TAG, "open fail"); goto fail0; }

    /* Query default format */
    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    ESP_RETURN_ON_FALSE(ioctl(s_video_fd, VIDIOC_G_FMT, &fmt) == 0, ESP_FAIL, TAG, "G_FMT fail");
    s_sensor_w    = (uint16_t)fmt.fmt.pix.width;
    s_sensor_h    = (uint16_t)fmt.fmt.pix.height;
    s_pixelformat = fmt.fmt.pix.pixelformat;
    /* fb_size = actual buffer length from driver (may differ from w*h*2 for packed RAW) */
    s_fb_size     = fmt.fmt.pix.sizeimage ? (uint32_t)fmt.fmt.pix.sizeimage
                                           : (uint32_t)(s_sensor_w * s_sensor_h * 2);
    ESP_LOGI(TAG, "Sensor: %dx%d fmt=0x%08lx size=%lu stride=%d",
             s_sensor_w, s_sensor_h, (unsigned long)s_pixelformat,
             (unsigned long)s_fb_size, (int)fmt.fmt.pix.bytesperline);

    /* Request buffers */
    struct v4l2_requestbuffers req = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .count = MAX_FB_NUM,
    };
    ESP_RETURN_ON_FALSE(ioctl(s_video_fd, VIDIOC_REQBUFS, &req) == 0, ESP_FAIL, TAG, "REQBUFS fail");
    s_nbufs = (int)req.count;

    for (int i = 0; i < s_nbufs; i++) {
        struct v4l2_buffer q = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = (uint32_t)i };
        ESP_RETURN_ON_FALSE(ioctl(s_video_fd, VIDIOC_QUERYBUF, &q) == 0, ESP_FAIL, TAG, "QUERYBUF fail");
        s_mmap_buf[i] = mmap(NULL, q.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_video_fd, q.m.offset);
        ESP_RETURN_ON_FALSE(s_mmap_buf[i] != MAP_FAILED, ESP_FAIL, TAG, "mmap fail");
    }
    for (int i = 0; i < s_nbufs; i++) {
        struct v4l2_buffer q = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = (uint32_t)i };
        ESP_RETURN_ON_FALSE(ioctl(s_video_fd, VIDIOC_QBUF, &q) == 0, ESP_FAIL, TAG, "QBUF fail");
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_RETURN_ON_FALSE(ioctl(s_video_fd, VIDIOC_STREAMON, &type) == 0, ESP_FAIL, TAG, "STREAMON fail");

    ESP_LOGI(TAG, "  [3/3] JPEG init...");
    ESP_RETURN_ON_FALSE(jpeg_init(quality) == ESP_OK, ESP_FAIL, TAG, "JPEG fail");

    s_frame_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(
        xTaskCreate(camera_task, "CamV4L2", CAM_TASK_STACK, NULL, CAM_TASK_PRIO, &s_cam_task) == pdPASS,
        ESP_FAIL, TAG, "task fail");

    ESP_LOGI(TAG, "Camera ready! %dx%d", s_sensor_w, s_sensor_h);
    return ESP_OK;

fail0:
    esp_video_deinit();
    return ESP_FAIL;
}

esp_err_t camera_module_get_frame(const uint8_t **jpeg_buf, size_t *jpeg_len)
{
    if (!s_jpeg_ready || !jpeg_buf || !jpeg_len) return ESP_ERR_NOT_FOUND;
    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;
    *jpeg_buf = s_jpeg_out;
    *jpeg_len = s_jpeg_out_len;
    xSemaphoreGive(s_frame_mutex);
    return ESP_OK;
}

esp_err_t camera_module_start(void) { return ESP_OK; }
esp_err_t camera_module_stop(void)
{
    if (s_video_fd >= 0) { int t = V4L2_BUF_TYPE_VIDEO_CAPTURE; ioctl(s_video_fd, VIDIOC_STREAMOFF, &t); close(s_video_fd); s_video_fd = -1; }
    if (s_jpeg_enc) { jpeg_del_encoder_engine(s_jpeg_enc); s_jpeg_enc = NULL; }
    if (s_jpeg_buf) { free(s_jpeg_buf); s_jpeg_buf = NULL; }
    if (s_jpeg_out) { free(s_jpeg_out); s_jpeg_out = NULL; }
    for (int i = 0; i < s_nbufs; i++) { if (s_mmap_buf[i] && s_mmap_buf[i] != MAP_FAILED) munmap(s_mmap_buf[i], s_fb_size); }
    if (s_frame_mutex) { vSemaphoreDelete(s_frame_mutex); s_frame_mutex = NULL; }
    if (s_cam_task) { vTaskDelete(s_cam_task); s_cam_task = NULL; }
    esp_video_deinit();
    ESP_LOGI(TAG, "Camera stopped");
    return ESP_OK;
}

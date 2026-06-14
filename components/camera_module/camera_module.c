/*
 * Camera Module — V4L2 MIPI-CSI camera + HW JPEG encode
 *
 * Pipeline: SC2336 → MIPI-CSI → /dev/video0 (RAW Bayer)
 * ISP pipeline controller disabled due to IPA AWB crash.
 * Fallback: encode RAW Bayer as grayscale JPEG (monochrome preview).
 *
 * Fixes (2026-06-10):
 *   - Double-buffer output to prevent HTTP/camera race (flickering)
 *   - Stride-aware: strip line padding when bytesperline > width (misalignment)
 *   - Check V4L2_BUF_FLAG_DONE (skip incomplete frames)
 *   - Use buf.bytesused for actual data size
 *
 * Fixes (2026-06-14):
 *   - Software AE: brightness sampling + V4L2 exposure/gain control
 *     Compensates for missing ISP auto-exposure (overexposed in bright light)
 *   - RAW10 support: 16bit/px V4L2 → 8bit/px extract for grayscale JPEG
 *   - Switched to 640×480 RAW10 @ 50fps (from 1280×720 RAW8 @ 30fps)
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
#include "esp_heap_caps.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "linux/videodev2.h"
#include "linux/v4l2-controls.h"
#include "esp_video_init.h"
#include "driver/jpeg_encode.h"
#include "camera_module.h"

#define TAG "Camera"

#define CAM_DEFAULT_QUALITY  55
#define CAM_TASK_STACK       8192
#define CAM_TASK_PRIO        5
#define JPEG_BUF_SIZE        (256 * 1024)
#define MAX_FB_NUM           3

/* ==================== Auto-Exposure (AE) ==================== */
#define AE_TARGET_BRIGHTNESS   40    /* 8-bit target (~15% of 255) */
#define AE_SAMPLE_SKIP         6     /* sample every Nth row and col (640px → 106×80) */
#define AE_FRAME_INTERVAL      10    /* run AE every N frames (50fps → 5Hz) */
#define AE_DAMPING             0.55f /* smoothing factor (0-1, lower=slower) */
#define AE_DEADBAND            8     /* brightness dead zone to prevent osc */
#define AE_FALLBACK_EXP_US     8000  /* fallback exposure in μs if query fails */
#define AE_GAIN_SMALL_STEP     8     /* gain index step for fine tuning */

/* ==================== State ==================== */
static int               s_video_fd = -1;
static jpeg_encoder_handle_t s_jpeg_enc = NULL;
static jpeg_encode_cfg_t s_jpeg_cfg;
static uint8_t          *s_jpeg_enc_buf = NULL;   /* HW encoder output buffer */
static size_t            s_jpeg_enc_buf_size = 0;

static uint8_t *s_mmap_buf[MAX_FB_NUM];
static uint32_t s_fb_stride = 0;      /* bytesperline from V4L2 */
static uint32_t s_fb_width  = 0;      /* actual image width */
static uint32_t s_fb_height = 0;      /* actual image height */
static int      s_nbufs = 0;
static uint32_t s_pixelformat = 0;    /* V4L2 FOURCC */

/* Stride-stripping / RAW10→8 conversion buffer */
static uint8_t  *s_row_buf = NULL;   /* for stride-stripping (RAW8) or 16→8bit (RAW10) */
static bool      s_is_raw10 = false; /* true if sensor outputs RAW10 */

static TaskHandle_t      s_cam_task = NULL;
static SemaphoreHandle_t s_frame_mutex = NULL;
static SemaphoreHandle_t s_frame_ready = NULL;  /* binary: posted when new frame ready */

/* ==================== Double-buffered JPEG output ====================
 * Camera task writes to s_jpeg_out[!s_published_idx], then swaps.
 * Reader reads from s_jpeg_out[s_published_idx] — never overwritten
 * until the next frame is fully encoded (≥1 frame interval).
 */
static uint8_t  *s_jpeg_out[2] = {NULL, NULL};
static uint32_t  s_jpeg_out_len[2] = {0, 0};
static volatile int s_published_idx = 0;  /* which buffer reader should use */
static volatile bool s_has_frame = false; /* at least one frame published */

/* AE state */
typedef struct {
    bool     active;            /* AE is initialized and running */
    int32_t  exp_100us;         /* current exposure in 100μs */
    int32_t  gain_idx;          /* current gain table index */
    int32_t  exp_min_100us;     /* min exposure */
    int32_t  exp_max_100us;     /* max exposure */
    int32_t  gain_min;          /* min gain index */
    int32_t  gain_max;          /* max gain index */
    float    smooth_brightness; /* EMA-smoothed brightness */
    uint32_t frame_count;       /* frame counter for interval */
} ae_state_t;

static ae_state_t s_ae = {
    .active = false,
    .exp_100us = AE_FALLBACK_EXP_US / 100,
    .gain_idx = 0,
    .exp_min_100us = 1,
    .exp_max_100us = AE_FALLBACK_EXP_US / 100,
    .gain_min = 0,
    .gain_max = 31,
    .smooth_brightness = AE_TARGET_BRIGHTNESS,
    .frame_count = 0,
};

/* ==================== Alloc helpers ==================== */
static uint8_t *cam_alloc(size_t size)
{
    uint8_t *p = heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!p) p = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    return p;
}

/* ==================== AE: Read/write V4L2 controls ==================== */
static int32_t ae_get_ctrl(uint32_t cid)
{
    struct v4l2_control ctrl = { .id = cid };
    if (ioctl(s_video_fd, VIDIOC_G_CTRL, &ctrl) == 0) return ctrl.value;
    return -1;
}

static esp_err_t ae_set_ctrl(uint32_t cid, int32_t value)
{
    struct v4l2_control ctrl = { .id = cid, .value = value };
    if (ioctl(s_video_fd, VIDIOC_S_CTRL, &ctrl) == 0) return ESP_OK;
    return ESP_FAIL;
}

static esp_err_t ae_query_ctrl(uint32_t cid, struct v4l2_queryctrl *q)
{
    q->id = cid;
    if (ioctl(s_video_fd, VIDIOC_QUERYCTRL, q) == 0) return ESP_OK;
    return ESP_FAIL;
}

/* ==================== AE: Brightness measurement ==================== */
static float ae_measure_brightness(const uint8_t *raw, uint32_t stride)
{
    uint64_t sum = 0;
    uint32_t count = 0;
    if (s_is_raw10) {
        /* RAW10: 16bit/px LE, sample byte[1] = bits[9:2] */
        for (uint32_t y = 0; y < s_fb_height; y += AE_SAMPLE_SKIP) {
            for (uint32_t x = 0; x < s_fb_width; x += AE_SAMPLE_SKIP) {
                sum += raw[y * stride + x * 2 + 1];
                count++;
            }
        }
    } else {
        for (uint32_t y = 0; y < s_fb_height; y += AE_SAMPLE_SKIP) {
            for (uint32_t x = 0; x < s_fb_width; x += AE_SAMPLE_SKIP) {
                sum += raw[y * stride + x];
                count++;
            }
        }
    }
    return (count > 0) ? (float)sum / (float)count : 0.0f;
}

/* ==================== AE: Init ==================== */
static void ae_init(void)
{
    /* Query exposure range */
    struct v4l2_queryctrl qc;
    if (ae_query_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE, &qc) == ESP_OK) {
        s_ae.exp_min_100us = qc.minimum;
        s_ae.exp_max_100us = qc.maximum;
    }
    /* Query gain range */
    if (ae_query_ctrl(V4L2_CID_GAIN, &qc) == ESP_OK) {
        s_ae.gain_min = 0;  /* gain is enum, always starts at 0 */
        s_ae.gain_max = (int32_t)qc.maximum;
    }
    /* Read current values */
    int32_t v = ae_get_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE);
    if (v >= 0) s_ae.exp_100us = v;
    v = ae_get_ctrl(V4L2_CID_GAIN);
    if (v >= 0) s_ae.gain_idx = v;

    s_ae.active = true;
    ESP_LOGI(TAG, "AE init: exp=%d (rng %d..%d), gain=%d (rng %d..%d)",
             (int)s_ae.exp_100us, (int)s_ae.exp_min_100us, (int)s_ae.exp_max_100us,
             (int)s_ae.gain_idx, (int)s_ae.gain_min, (int)s_ae.gain_max);
}

/* ==================== AE: Main loop ==================== */
static void ae_run(const uint8_t *raw, uint32_t stride)
{
    if (!s_ae.active) return;

    s_ae.frame_count++;
    if (s_ae.frame_count % AE_FRAME_INTERVAL != 0) return;

    /* Measure brightness */
    float measured = ae_measure_brightness(raw, stride);
    if (measured < 1.0f) return; /* too dark to measure reliably */

    /* EMA smooth */
    s_ae.smooth_brightness = AE_DAMPING * measured + (1.0f - AE_DAMPING) * s_ae.smooth_brightness;

    /* Dead band check */
    float err = AE_TARGET_BRIGHTNESS - s_ae.smooth_brightness;
    if (err > -AE_DEADBAND && err < AE_DEADBAND) return;

    int32_t new_exp = s_ae.exp_100us;
    int32_t new_gain = s_ae.gain_idx;

    if (err < 0) {
        /* Too bright — reduce exposure first, then gain */
        float ratio = AE_TARGET_BRIGHTNESS / s_ae.smooth_brightness;
        if (new_exp > (s_ae.exp_min_100us + 1)) {
            new_exp = (int32_t)(new_exp * ratio);
            if (new_exp < s_ae.exp_min_100us) new_exp = s_ae.exp_min_100us;
            if (new_exp == s_ae.exp_min_100us && s_ae.smooth_brightness > AE_TARGET_BRIGHTNESS + AE_DEADBAND) {
                /* Still too bright at min exposure — reduce gain */
                new_gain = s_ae.gain_idx - AE_GAIN_SMALL_STEP;
            }
        } else {
            new_gain = s_ae.gain_idx - AE_GAIN_SMALL_STEP;
        }
    } else {
        /* Too dark — increase gain first, then exposure */
        if (new_gain < s_ae.gain_max) {
            new_gain = s_ae.gain_idx + AE_GAIN_SMALL_STEP;
        } else {
            float ratio = AE_TARGET_BRIGHTNESS / s_ae.smooth_brightness;
            new_exp = (int32_t)(new_exp * ratio);
        }
    }

    /* Clamp */
    if (new_exp < s_ae.exp_min_100us) new_exp = s_ae.exp_min_100us;
    if (new_exp > s_ae.exp_max_100us) new_exp = s_ae.exp_max_100us;
    if (new_gain < s_ae.gain_min) new_gain = s_ae.gain_min;
    if (new_gain > s_ae.gain_max) new_gain = s_ae.gain_max;

    /* Apply if changed */
    bool changed = false;
    if (new_exp != s_ae.exp_100us) {
        if (ae_set_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE, new_exp) == ESP_OK) {
            s_ae.exp_100us = new_exp;
            changed = true;
        }
    }
    if (new_gain != s_ae.gain_idx) {
        if (ae_set_ctrl(V4L2_CID_GAIN, new_gain) == ESP_OK) {
            s_ae.gain_idx = new_gain;
            changed = true;
        }
    }

    if (changed && s_ae.frame_count % (AE_FRAME_INTERVAL * 10) == 0) {
        ESP_LOGI(TAG, "AE: bright=%.0f exp=%d gain=%d",
                 (double)s_ae.smooth_brightness,
                 (int)s_ae.exp_100us,
                 (int)s_ae.gain_idx);
    }
}

/* ==================== JPEG Init ==================== */
static esp_err_t jpeg_init(uint8_t quality)
{
    jpeg_encode_engine_cfg_t eng_cfg = { .timeout_ms = 5000 };
    ESP_RETURN_ON_ERROR(jpeg_new_encoder_engine(&eng_cfg, &s_jpeg_enc), TAG, "JPEG engine");

    jpeg_encode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    size_t allocated = 0;
    s_jpeg_enc_buf = (uint8_t *)jpeg_alloc_encoder_mem(JPEG_BUF_SIZE, &mem_cfg, &allocated);
    if (!s_jpeg_enc_buf) { ESP_LOGE(TAG, "JPEG buf fail"); return ESP_ERR_NO_MEM; }
    s_jpeg_enc_buf_size = allocated;

    /* If driver gives us RAW Bayer, encode as grayscale. */
    bool is_raw = (s_pixelformat == V4L2_PIX_FMT_SBGGR8 ||
                   s_pixelformat == V4L2_PIX_FMT_SGBRG8 ||
                   s_pixelformat == V4L2_PIX_FMT_SGRBG8 ||
                   s_pixelformat == V4L2_PIX_FMT_SRGGB8 ||
                   s_pixelformat == V4L2_PIX_FMT_SBGGR10 ||
                   s_pixelformat == V4L2_PIX_FMT_SGBRG10 ||
                   s_pixelformat == V4L2_PIX_FMT_SGRBG10 ||
                   s_pixelformat == V4L2_PIX_FMT_SRGGB10);

    s_is_raw10 = (s_pixelformat == V4L2_PIX_FMT_SBGGR10 ||
                  s_pixelformat == V4L2_PIX_FMT_SGBRG10 ||
                  s_pixelformat == V4L2_PIX_FMT_SGRBG10 ||
                  s_pixelformat == V4L2_PIX_FMT_SRGGB10);

    s_jpeg_cfg.height        = s_fb_height;
    s_jpeg_cfg.width         = s_fb_width;
    s_jpeg_cfg.image_quality = quality;

    if (is_raw) {
        s_jpeg_cfg.src_type   = JPEG_ENCODE_IN_FORMAT_GRAY;
        s_jpeg_cfg.sub_sample = JPEG_DOWN_SAMPLING_GRAY;
        if (s_is_raw10) {
            ESP_LOGW(TAG, "JPEG: RAW10→8bit→grayscale (no ISP)");
        } else {
            ESP_LOGW(TAG, "JPEG: RAW Bayer → grayscale (no ISP)");
        }
    } else {
        s_jpeg_cfg.src_type   = JPEG_ENCODE_IN_FORMAT_RGB565;
        s_jpeg_cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV422;
    }
    ESP_LOGI(TAG, "JPEG: %" PRIu32 "x%" PRIu32 " Q=%d fmt=0x%08lx (%s) stride=%" PRIu32,
             s_fb_width, s_fb_height, quality, (unsigned long)s_pixelformat,
             is_raw ? "RAW→GRAY" : "RGB565", s_fb_stride);
    return ESP_OK;
}

/* ==================== Camera Task ==================== */
static void camera_task(void *arg)
{
    uint32_t fc = 0;
    int write_idx = 1;  /* start writing to buffer[1] */
    ESP_LOGI(TAG, "Cam task running (%" PRIu32 "x%" PRIu32 " stride=%" PRIu32 ")",
             s_fb_width, s_fb_height, s_fb_stride);

    while (1) {
        /* ── DQBUF ── */
        struct v4l2_buffer vbuf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(s_video_fd, VIDIOC_DQBUF, &vbuf) != 0) {
            vTaskDelay(pdMS_TO_TICKS(10)); continue;
        }

        /* Skip incomplete frames (official example pattern) */
        if (!(vbuf.flags & V4L2_BUF_FLAG_DONE)) {
            ioctl(s_video_fd, VIDIOC_QBUF, &vbuf);
            continue;
        }

        uint8_t *src = s_mmap_buf[vbuf.index];
        if (!src) goto qbuf;

        /* ── Pixel format handling ──
         * RAW10: V4L2 → 16bit/px (LE) → extract byte[1] → 8bit/px for JPEG
         * RAW8:  bytesperline may be > width (padding) → strip to w*h
         */
        uint8_t *enc_src = src;
        uint32_t enc_src_size = vbuf.bytesused;

        if (s_is_raw10 && s_row_buf) {
            /* RAW10: extract 8-bit from 16-bit samples.
             * Left-aligned in LE:  byte[0]=P[0:7]lo, byte[1]=P[9:2]hi.
             * Taking byte[1] gives usable 8-bit brightness. */
            uint8_t *dst = s_row_buf;
            for (uint32_t row = 0; row < s_fb_height; row++) {
                const uint8_t *line = src + row * s_fb_stride;
                for (uint32_t col = 0; col < s_fb_width; col++) {
                    dst[row * s_fb_width + col] = line[col * 2 + 1];
                }
            }
            enc_src = s_row_buf;
            enc_src_size = s_fb_width * s_fb_height;
        } else if (s_fb_stride > s_fb_width && s_row_buf) {
            /* Strip padding: copy each row without gaps */
            for (uint32_t row = 0; row < s_fb_height; row++) {
                memcpy(s_row_buf + row * s_fb_width,
                       src + row * s_fb_stride,
                       s_fb_width);
            }
            enc_src = s_row_buf;
            enc_src_size = s_fb_width * s_fb_height;
        }

        /* ── HW JPEG Encode ── */
        uint32_t out_len = 0;
        esp_err_t ret = jpeg_encoder_process(
            s_jpeg_enc, &s_jpeg_cfg,
            enc_src, enc_src_size,
            s_jpeg_enc_buf, (uint32_t)s_jpeg_enc_buf_size, &out_len);

        if (ret == ESP_OK && out_len > 0 && out_len <= JPEG_BUF_SIZE) {
            /* Allocate/lazy-init the write-target double-buffer */
            if (!s_jpeg_out[write_idx]) {
                s_jpeg_out[write_idx] = cam_alloc(JPEG_BUF_SIZE);
            }
            if (s_jpeg_out[write_idx]) {
                if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    memcpy(s_jpeg_out[write_idx], s_jpeg_enc_buf, out_len);
                    s_jpeg_out_len[write_idx] = out_len;

                    /* Atomic swap: publish this frame */
                    s_published_idx = write_idx;
                    s_has_frame = true;

                    /* Toggle write target for next frame */
                    write_idx = 1 - write_idx;
                    xSemaphoreGive(s_frame_mutex);

                    /* Signal waiters (drain stale first — at most 1 pending) */
                    xSemaphoreTake(s_frame_ready, 0);
                    xSemaphoreGive(s_frame_ready);
                }
            }

            if (++fc % 50 == 0) {
                ESP_LOGI(TAG, "F#%" PRIu32 ": %" PRIu32 "B bytesused=%" PRIu32,
                         fc, out_len, (uint32_t)vbuf.bytesused);
            }
        } else if (ret != ESP_OK && fc < 5) {
            ESP_LOGW(TAG, "JPEG encode err: %s", esp_err_to_name(ret));
        }

        /* ── Run AE on this frame's raw data ── */
        ae_run(src, s_fb_stride);

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
    s_fb_width     = (uint32_t)fmt.fmt.pix.width;
    s_fb_height    = (uint32_t)fmt.fmt.pix.height;
    s_pixelformat  = fmt.fmt.pix.pixelformat;
    s_fb_stride    = fmt.fmt.pix.bytesperline ? (uint32_t)fmt.fmt.pix.bytesperline
                                               : s_fb_width;
    ESP_LOGI(TAG, "Sensor: %" PRIu32 "x%" PRIu32 " fmt=0x%08lx stride=%" PRIu32 " sizeimage=%lu",
             s_fb_width, s_fb_height,
             (unsigned long)s_pixelformat,
             s_fb_stride,
             (unsigned long)fmt.fmt.pix.sizeimage);

    /* Allocate stride-stripping / RAW10→8 conversion buffer */
    bool is_raw10 = (s_pixelformat == V4L2_PIX_FMT_SBGGR10 ||
                     s_pixelformat == V4L2_PIX_FMT_SGBRG10 ||
                     s_pixelformat == V4L2_PIX_FMT_SGRBG10 ||
                     s_pixelformat == V4L2_PIX_FMT_SRGGB10);

    if (is_raw10) {
        /* RAW10: 16bit/px V4L2 → 8bit/px for grayscale JPEG */
        s_row_buf = cam_alloc(s_fb_width * s_fb_height);
        if (s_row_buf) {
            ESP_LOGI(TAG, "RAW10→8 conv buf: %" PRIu32 " bytes", (uint32_t)(s_fb_width * s_fb_height));
        } else {
            ESP_LOGE(TAG, "Failed alloc RAW10 conv buf (%" PRIu32 " bytes)",
                     (uint32_t)(s_fb_width * s_fb_height));
        }
    } else if (s_fb_stride > s_fb_width) {
        s_row_buf = cam_alloc(s_fb_width * s_fb_height);
        if (s_row_buf) {
            ESP_LOGW(TAG, "Stride padding detected (%" PRIu32 " > %" PRIu32 ") — stripping each row",
                     s_fb_stride, s_fb_width);
        } else {
            ESP_LOGE(TAG, "Failed to alloc stride-strip buffer (%" PRIu32 " bytes)",
                     (uint32_t)(s_fb_width * s_fb_height));
        }
    }

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

    /* Init output buffers */
    s_jpeg_out[0] = cam_alloc(JPEG_BUF_SIZE);
    s_jpeg_out[1] = cam_alloc(JPEG_BUF_SIZE);
    if (!s_jpeg_out[0] || !s_jpeg_out[1]) {
        ESP_LOGW(TAG, "Double-buffer alloc fail, will lazy-init");
    }

    s_frame_mutex = xSemaphoreCreateMutex();
    s_frame_ready = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_frame_mutex && s_frame_ready, ESP_ERR_NO_MEM, TAG, "sem fail");
    /* s_frame_ready starts at 0 — first frame will give it */

    ESP_RETURN_ON_FALSE(
        xTaskCreate(camera_task, "CamV4L2", CAM_TASK_STACK, NULL, CAM_TASK_PRIO, &s_cam_task) == pdPASS,
        ESP_FAIL, TAG, "task fail");

    /* Init auto-exposure (after sensor is streaming) */
    vTaskDelay(pdMS_TO_TICKS(50));
    ae_init();

    ESP_LOGI(TAG, "Camera ready! %" PRIu32 "x%" PRIu32 " stride=%" PRIu32,
             s_fb_width, s_fb_height, s_fb_stride);
    return ESP_OK;

fail0:
    esp_video_deinit();
    return ESP_FAIL;
}

esp_err_t camera_module_get_frame(const uint8_t **jpeg_buf, size_t *jpeg_len)
{
    if (!jpeg_buf || !jpeg_len) return ESP_ERR_INVALID_ARG;

    /* Wait for a new frame (short timeout, then fallback to last-known) */
    xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(50)); /* ignore result — try last-known on timeout */

    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        /* Mutex held too long by camera task — last resort */
        if (!s_has_frame || !s_jpeg_out[s_published_idx]) return ESP_ERR_TIMEOUT;
        *jpeg_buf = s_jpeg_out[s_published_idx];
        *jpeg_len = s_jpeg_out_len[s_published_idx];
        return ESP_OK;
    }

    int idx = s_published_idx;
    *jpeg_buf = s_jpeg_out[idx];
    *jpeg_len = s_jpeg_out_len[idx];
    /* Safe: camera task writes to 1-idx, this buffer won't be touched
     * until the NEXT frame is fully encoded. */

    xSemaphoreGive(s_frame_mutex);
    return (*jpeg_buf && *jpeg_len) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t camera_module_start(void) { return ESP_OK; }

esp_err_t camera_module_stop(void)
{
    if (s_video_fd >= 0) {
        int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(s_video_fd, VIDIOC_STREAMOFF, &t); close(s_video_fd); s_video_fd = -1;
    }
    if (s_jpeg_enc) { jpeg_del_encoder_engine(s_jpeg_enc); s_jpeg_enc = NULL; }
    if (s_jpeg_enc_buf) { free(s_jpeg_enc_buf); s_jpeg_enc_buf = NULL; }
    for (int i = 0; i < 2; i++) {
        if (s_jpeg_out[i]) { free(s_jpeg_out[i]); s_jpeg_out[i] = NULL; }
    }
    if (s_row_buf) { free(s_row_buf); s_row_buf = NULL; }
    for (int i = 0; i < s_nbufs; i++) {
        if (s_mmap_buf[i] && s_mmap_buf[i] != MAP_FAILED) munmap(s_mmap_buf[i], s_fb_stride * s_fb_height);
    }
    if (s_frame_mutex) { vSemaphoreDelete(s_frame_mutex); s_frame_mutex = NULL; }
    if (s_frame_ready) { vSemaphoreDelete(s_frame_ready); s_frame_ready = NULL; }
    if (s_cam_task) { vTaskDelete(s_cam_task); s_cam_task = NULL; }
    esp_video_deinit();
    ESP_LOGI(TAG, "Camera stopped");
    return ESP_OK;
}

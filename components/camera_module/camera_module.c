/*
 * 摄像头模块 — V4L2 MIPI-CSI 摄像头 + 硬件 JPEG 编码
 *
 * 管线: SC2336 → MIPI-CSI → ISP (/dev/video0 RGB565) → 硬件 JPEG
 * ISP 将 RAW Bayer 自动转换为 RGB565。
 * ISP 管线控制器 (3A) 已禁用，以避免 IPA AWB 崩溃。
 *
 * 我们通过 V4L2 控件在 ISP RGB565 输出上运行软件 AE。
 *
 * 修复记录 (2026-06-10):
 *   - 双缓冲输出，防止 HTTP/摄像头竞争 (画面撕裂)
 *   - 步长感知: 当 bytesperline > width 时剥离行填充 (错位)
 *   - 检查 V4L2_BUF_FLAG_DONE (跳过不完整帧)
 *   - 使用 buf.bytesused 作为实际数据大小
 *
 * 修复记录 (2026-06-14):
 *   - 软件 AE: 亮度采样 + V4L2 曝光/增益控制
 *   - RGB565 支持: ISP 输出路径 (多字节步长修复, 绿通道 AE)
 *   - 切换到 640×480 RAW10 @ 50fps (原为 1280×720 RAW8 @ 30fps)
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
#include "esp_timer.h"
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

/* ==================== 自动曝光 (AE) ==================== */
#define AE_TARGET_BRIGHTNESS   40    /* 8-bit 目标 (~15% of 255) */
#define AE_SAMPLE_SKIP         6     /* 每隔 N 行和列采样 (640px → 106×80) */
#define AE_FRAME_INTERVAL      10    /* 每隔 N 帧运行 AE (50fps → 5Hz) */
#define AE_DAMPING             0.55f /* 平滑因子 (0-1, 越小越慢) */
#define AE_DEADBAND            8     /* 亮度死区，防止震荡 */
#define AE_FALLBACK_EXP_US     8000  /* 查询失败时的回退曝光值 (μs) */
#define AE_GAIN_SMALL_STEP     8     /* 微调用的增益索引步进 */

/* ==================== 状态 ==================== */
static int               s_video_fd = -1;
static jpeg_encoder_handle_t s_jpeg_enc = NULL;
static jpeg_encode_cfg_t s_jpeg_cfg;
static uint8_t          *s_jpeg_enc_buf = NULL;   /* 硬件编码器输出缓冲区 */
static size_t            s_jpeg_enc_buf_size = 0;

static uint8_t *s_mmap_buf[MAX_FB_NUM];
static uint32_t s_fb_stride = 0;      /* V4L2 的 bytesperline */
static uint32_t s_fb_width  = 0;      /* 实际图像宽度 */
static uint32_t s_fb_height = 0;      /* 实际图像高度 */
static int      s_nbufs = 0;
static uint32_t s_pixelformat = 0;    /* V4L2 FOURCC */

/* 步长剥离 / RAW10→8 转换缓冲区 */
static uint8_t  *s_row_buf = NULL;   /* 步长剥离(RAW8) 或 16→8bit(RAW10) 用 */
static bool      s_is_raw10 = false; /* 传感器输出 RAW10 时为 true */

static TaskHandle_t      s_cam_task = NULL;
static SemaphoreHandle_t s_frame_mutex = NULL;
static SemaphoreHandle_t s_frame_ready = NULL;  /* 二进制信号量: 新帧就绪时发布 */

/* ==================== 双缓冲 JPEG 输出 ====================
 * 摄像头任务写入 s_jpeg_out[!s_published_idx]，然后交换。
 * 读取方读取 s_jpeg_out[s_published_idx] — 在下一帧编码完成前不会被覆盖
 * (间隔 ≥1 帧周期)。
 */
static uint8_t  *s_jpeg_out[2] = {NULL, NULL};
static uint32_t  s_jpeg_out_len[2] = {0, 0};
static volatile int s_published_idx = 0;  /* 读取方应使用的缓冲区索引 */
static volatile bool s_has_frame = false; /* 至少有一帧已发布 */

/* AE 状态 */
typedef struct {
    bool     active;            /* AE 已初始化并运行 */
    int32_t  exp_100us;         /* 当前曝光值 (100μs) */
    int32_t  gain_idx;          /* 当前增益表索引 */
    int32_t  exp_min_100us;     /* 最小曝光值 */
    int32_t  exp_max_100us;     /* 最大曝光值 */
    int32_t  gain_min;          /* 最小增益索引 */
    int32_t  gain_max;          /* 最大增益索引 */
    float    smooth_brightness; /* EMA 平滑亮度 */
    uint32_t frame_count;       /* 帧计数器 (用于间隔控制) */
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

/* FPS 计量 */
static volatile uint32_t s_fps_counter = 0;
static int64_t            s_fps_last_us = 0;
static volatile uint32_t s_cached_fps = 0;

/* ==================== 内存分配辅助 ==================== */
static uint8_t *cam_alloc(size_t size)
{
    uint8_t *p = heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!p) p = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    return p;
}

/* ==================== AE: 读取/写入 V4L2 控件 ==================== */
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

/* ==================== AE: 亮度测量 ==================== */
static float ae_measure_brightness(const uint8_t *raw, uint32_t stride)
{
    uint64_t sum = 0;
    uint32_t count = 0;
    bool is_rgb565 = (s_pixelformat == V4L2_PIX_FMT_RGB565);

    if (is_rgb565) {
        /* RGB565: 2字节/像素。提取绿色通道 (6位 → 8位量程) */
        for (uint32_t y = 0; y < s_fb_height; y += AE_SAMPLE_SKIP) {
            const uint8_t *line = raw + y * stride;
            for (uint32_t x = 0; x < s_fb_width; x += AE_SAMPLE_SKIP) {
                uint8_t lo = line[x * 2];
                uint8_t hi = line[x * 2 + 1];
                /* 绿色: RGB565 的 bit[10:5] = (hi[2:0] << 3) | (lo[7:5]) */
                uint8_t g6 = ((hi & 0x07) << 3) | (lo >> 5);
                sum += (g6 << 2); /* 6-bit → 8-bit */
                count++;
            }
        }
    } else if (s_is_raw10) {
        /* RAW10: 16bit/px LE, 采样 byte[1] = bits[9:2] */
        for (uint32_t y = 0; y < s_fb_height; y += AE_SAMPLE_SKIP) {
            for (uint32_t x = 0; x < s_fb_width; x += AE_SAMPLE_SKIP) {
                sum += raw[y * stride + x * 2 + 1];
                count++;
            }
        }
    } else {
        /* RAW8 / 灰度: 1字节/像素 */
        for (uint32_t y = 0; y < s_fb_height; y += AE_SAMPLE_SKIP) {
            for (uint32_t x = 0; x < s_fb_width; x += AE_SAMPLE_SKIP) {
                sum += raw[y * stride + x];
                count++;
            }
        }
    }
    return (count > 0) ? (float)sum / (float)count : 0.0f;
}

/* ==================== AE: 初始化 ==================== */
static void ae_init(void)
{
    /* 查询曝光范围 */
    struct v4l2_queryctrl qc;
    if (ae_query_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE, &qc) == ESP_OK) {
        s_ae.exp_min_100us = qc.minimum;
        s_ae.exp_max_100us = qc.maximum;
    }
    /* 查询增益范围 */
    if (ae_query_ctrl(V4L2_CID_GAIN, &qc) == ESP_OK) {
        s_ae.gain_min = 0;  /* 增益是枚举类型，始终从 0 开始 */
        s_ae.gain_max = (int32_t)qc.maximum;
    }
    /* 读取当前值 */
    int32_t v = ae_get_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE);
    if (v >= 0) s_ae.exp_100us = v;
    v = ae_get_ctrl(V4L2_CID_GAIN);
    if (v >= 0) s_ae.gain_idx = v;

    s_ae.active = true;
    ESP_LOGI(TAG, "AE 已初始化: exp=%d (范围 %d..%d), gain=%d (范围 %d..%d)",
             (int)s_ae.exp_100us, (int)s_ae.exp_min_100us, (int)s_ae.exp_max_100us,
             (int)s_ae.gain_idx, (int)s_ae.gain_min, (int)s_ae.gain_max);
}

/* ==================== AE: 主循环 ==================== */
static void ae_run(const uint8_t *raw, uint32_t stride)
{
    if (!s_ae.active) return;

    s_ae.frame_count++;
    if (s_ae.frame_count % AE_FRAME_INTERVAL != 0) return;

    /* 测量亮度 */
    float measured = ae_measure_brightness(raw, stride);
    if (measured < 1.0f) return; /* 太暗无法可靠测量 */

    /* EMA 平滑 */
    s_ae.smooth_brightness = AE_DAMPING * measured + (1.0f - AE_DAMPING) * s_ae.smooth_brightness;

    /* 死区检查 */
    float err = AE_TARGET_BRIGHTNESS - s_ae.smooth_brightness;
    if (err > -AE_DEADBAND && err < AE_DEADBAND) return;

    int32_t new_exp = s_ae.exp_100us;
    int32_t new_gain = s_ae.gain_idx;

    if (err < 0) {
        /* 太亮 — 先降曝光，再降增益 */
        float ratio = AE_TARGET_BRIGHTNESS / s_ae.smooth_brightness;
        if (new_exp > (s_ae.exp_min_100us + 1)) {
            new_exp = (int32_t)(new_exp * ratio);
            if (new_exp < s_ae.exp_min_100us) new_exp = s_ae.exp_min_100us;
            if (new_exp == s_ae.exp_min_100us && s_ae.smooth_brightness > AE_TARGET_BRIGHTNESS + AE_DEADBAND) {
                /* 最小曝光仍然过亮 — 降低增益 */
                new_gain = s_ae.gain_idx - AE_GAIN_SMALL_STEP;
            }
        } else {
            new_gain = s_ae.gain_idx - AE_GAIN_SMALL_STEP;
        }
    } else {
        /* 太暗 — 先升增益，再升曝光 */
        if (new_gain < s_ae.gain_max) {
            new_gain = s_ae.gain_idx + AE_GAIN_SMALL_STEP;
        } else {
            float ratio = AE_TARGET_BRIGHTNESS / s_ae.smooth_brightness;
            new_exp = (int32_t)(new_exp * ratio);
        }
    }

    /* 钳位 */
    if (new_exp < s_ae.exp_min_100us) new_exp = s_ae.exp_min_100us;
    if (new_exp > s_ae.exp_max_100us) new_exp = s_ae.exp_max_100us;
    if (new_gain < s_ae.gain_min) new_gain = s_ae.gain_min;
    if (new_gain > s_ae.gain_max) new_gain = s_ae.gain_max;

    /* 有变化则应用 */
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
        ESP_LOGI(TAG, "AE: 亮度=%.0f 曝光=%d 增益=%d",
                 (double)s_ae.smooth_brightness,
                 (int)s_ae.exp_100us,
                 (int)s_ae.gain_idx);
    }
    /* 定期状态输出 (~10s一次)，即使稳定不变 */
    if (s_ae.frame_count % (AE_FRAME_INTERVAL * 100) == 0) {
        ESP_LOGI(TAG, "AE 状态: 亮度=%.0f 曝光=%d/%d 增益=%d/%d",
                 (double)s_ae.smooth_brightness,
                 (int)s_ae.exp_100us, (int)s_ae.exp_max_100us,
                 (int)s_ae.gain_idx, (int)s_ae.gain_max);
    }
}

/* ==================== JPEG 初始化 ==================== */
static esp_err_t jpeg_init(uint8_t quality)
{
    jpeg_encode_engine_cfg_t eng_cfg = { .timeout_ms = 5000 };
    ESP_RETURN_ON_ERROR(jpeg_new_encoder_engine(&eng_cfg, &s_jpeg_enc), TAG, "JPEG 引擎");

    jpeg_encode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    size_t allocated = 0;
    s_jpeg_enc_buf = (uint8_t *)jpeg_alloc_encoder_mem(JPEG_BUF_SIZE, &mem_cfg, &allocated);
    if (!s_jpeg_enc_buf) { ESP_LOGE(TAG, "JPEG 缓冲区分配失败"); return ESP_ERR_NO_MEM; }
    s_jpeg_enc_buf_size = allocated;

    /* 如果驱动给出 RAW Bayer，编码为灰度 */
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
            ESP_LOGW(TAG, "JPEG: RAW10→8bit→灰度 (无 ISP)");
        } else {
            ESP_LOGW(TAG, "JPEG: RAW Bayer → 灰度 (无 ISP)");
        }
    } else {
        s_jpeg_cfg.src_type   = JPEG_ENCODE_IN_FORMAT_RGB565;
        s_jpeg_cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV422;
    }
    ESP_LOGI(TAG, "JPEG: %" PRIu32 "x%" PRIu32 " Q=%d 格式=0x%08lx (%s) 步长=%" PRIu32,
             s_fb_width, s_fb_height, quality, (unsigned long)s_pixelformat,
             is_raw ? "RAW→灰度" : "RGB565", s_fb_stride);
    return ESP_OK;
}

/* ==================== 摄像头任务 ==================== */
static void camera_task(void *arg)
{
    uint32_t fc = 0;
    int write_idx = 1;  /* 起始写入缓冲区[1] */
    ESP_LOGI(TAG, "摄像头任务运行中 (%" PRIu32 "x%" PRIu32 " 步长=%" PRIu32 ")",
             s_fb_width, s_fb_height, s_fb_stride);

    while (1) {
        /* ── DQBUF ── */
        struct v4l2_buffer vbuf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(s_video_fd, VIDIOC_DQBUF, &vbuf) != 0) {
            vTaskDelay(pdMS_TO_TICKS(10)); continue;
        }

        /* 跳过不完整帧 (官方示例模式) */
        if (!(vbuf.flags & V4L2_BUF_FLAG_DONE)) {
            ioctl(s_video_fd, VIDIOC_QBUF, &vbuf);
            continue;
        }

        uint8_t *src = s_mmap_buf[vbuf.index];
        if (!src) goto qbuf;

        /* ── 像素格式处理 ──
         * RAW10: V4L2 → 16bit/px (LE) → 提取 byte[1] → 8bit/px 供 JPEG 使用
         * RAW8:  bytesperline 可能大于 width (填充) → 剥离到 w*h
         * RGB565: 直接透传 (ISP 已处理)
         */
        uint8_t *enc_src = src;
        uint32_t enc_src_size = vbuf.bytesused;

        bool is_rgb565 = (s_pixelformat == V4L2_PIX_FMT_RGB565);

        if (s_is_raw10 && s_row_buf) {
            /* RAW10→8bit: 从 16-bit 采样中提取高字节 */
            uint8_t *dst = s_row_buf;
            for (uint32_t row = 0; row < s_fb_height; row++) {
                const uint8_t *line = src + row * s_fb_stride;
                for (uint32_t col = 0; col < s_fb_width; col++) {
                    dst[row * s_fb_width + col] = line[col * 2 + 1];
                }
            }
            enc_src = s_row_buf;
            enc_src_size = s_fb_width * s_fb_height;
        } else if (!is_rgb565 && s_fb_stride > s_fb_width && s_row_buf) {
            /* RAW8 去除填充: 逐行无间隙复制 */
            for (uint32_t row = 0; row < s_fb_height; row++) {
                memcpy(s_row_buf + row * s_fb_width,
                       src + row * s_fb_stride,
                       s_fb_width);
            }
            enc_src = s_row_buf;
            enc_src_size = s_fb_width * s_fb_height;
        }

        /* ── 硬件 JPEG 编码 ── */
        uint32_t out_len = 0;
        esp_err_t ret = jpeg_encoder_process(
            s_jpeg_enc, &s_jpeg_cfg,
            enc_src, enc_src_size,
            s_jpeg_enc_buf, (uint32_t)s_jpeg_enc_buf_size, &out_len);

        if (ret == ESP_OK && out_len > 0 && out_len <= JPEG_BUF_SIZE) {
            /* 尽早增加 FPS 计数器 (双缓冲拷贝之前) */
            s_fps_counter++;

            /* 分配/延迟初始化写入目标双缓冲区 */
            if (!s_jpeg_out[write_idx]) {
                s_jpeg_out[write_idx] = cam_alloc(JPEG_BUF_SIZE);
            }
            if (s_jpeg_out[write_idx]) {
                if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    memcpy(s_jpeg_out[write_idx], s_jpeg_enc_buf, out_len);
                    s_jpeg_out_len[write_idx] = out_len;

                    /* 原子交换: 发布本帧 */
                    s_published_idx = write_idx;
                    s_has_frame = true;

                    /* 切换写入目标到另一缓冲区供下一帧使用 */
                    write_idx = 1 - write_idx;
                    xSemaphoreGive(s_frame_mutex);

                    /* 通知等待方 (先排空过期信号 — 最多 1 个待处理) */
                    xSemaphoreTake(s_frame_ready, 0);
                    xSemaphoreGive(s_frame_ready);
                }
            }

            if (++fc % 50 == 0) {
                ESP_LOGI(TAG, "帧#%" PRIu32 ": %" PRIu32 "B bytesused=%" PRIu32,
                         fc, out_len, (uint32_t)vbuf.bytesused);
            }
        } else if (ret != ESP_OK && fc < 5) {
            ESP_LOGW(TAG, "JPEG 编码错误: %s", esp_err_to_name(ret));
        }

        /* ── 对本帧原始数据运行 AE ── */
        ae_run(src, s_fb_stride);

    qbuf:
        ioctl(s_video_fd, VIDIOC_QBUF, &vbuf);
    }
}

/* ==================== 公开 API ==================== */
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
    ESP_RETURN_ON_ERROR(esp_video_init(&vcfg), TAG, "esp_video_init 失败");

    ESP_LOGI(TAG, "  [2/3] V4L2 设置...");
    s_video_fd = open("/dev/video0", O_RDWR);
    if (s_video_fd < 0) { ESP_LOGE(TAG, "open 失败"); goto fail0; }

    /* 查询默认格式 */
    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    ESP_RETURN_ON_FALSE(ioctl(s_video_fd, VIDIOC_G_FMT, &fmt) == 0, ESP_FAIL, TAG, "G_FMT 失败");
    s_fb_width     = (uint32_t)fmt.fmt.pix.width;
    s_fb_height    = (uint32_t)fmt.fmt.pix.height;
    s_pixelformat  = fmt.fmt.pix.pixelformat;
    s_fb_stride    = fmt.fmt.pix.bytesperline ? (uint32_t)fmt.fmt.pix.bytesperline
                                               : s_fb_width;
    /* 修复: 多字节格式 (RGB565=2B/px) — V4L2 驱动可能将 bytesperline 留为 0 */
    if (s_pixelformat == V4L2_PIX_FMT_RGB565) {
        if (s_fb_stride < s_fb_width * 2) s_fb_stride = s_fb_width * 2;
    }
    ESP_LOGI(TAG, "传感器: %" PRIu32 "x%" PRIu32 " 格式=0x%08lx 步长=%" PRIu32 " sizeimage=%lu",
             s_fb_width, s_fb_height,
             (unsigned long)s_pixelformat,
             s_fb_stride,
             (unsigned long)fmt.fmt.pix.sizeimage);

    /* 尽早检测像素格式类型 */
    bool is_rgb565  = (s_pixelformat == V4L2_PIX_FMT_RGB565);
    bool is_raw10 = (s_pixelformat == V4L2_PIX_FMT_SBGGR10 ||
                     s_pixelformat == V4L2_PIX_FMT_SGBRG10 ||
                     s_pixelformat == V4L2_PIX_FMT_SGRBG10 ||
                     s_pixelformat == V4L2_PIX_FMT_SRGGB10);

    /* 分配转换/剥离缓冲区 */
    if (is_raw10) {
        /* RAW10: 16bit/px V4L2 → 需要 8bit/px 缓冲区供灰度 JPEG */
        s_row_buf = cam_alloc(s_fb_width * s_fb_height);
        if (s_row_buf) {
            ESP_LOGI(TAG, "RAW10→8 转换缓冲: %" PRIu32 " 字节", (uint32_t)(s_fb_width * s_fb_height));
        } else {
            ESP_LOGE(TAG, "分配 RAW10 转换缓冲区失败");
        }
    } else if (!is_rgb565 && s_fb_stride > s_fb_width) {
        /* RAW8 有填充: 逐行剥离 */
        s_row_buf = cam_alloc(s_fb_width * s_fb_height);
        if (s_row_buf) {
            ESP_LOGW(TAG, "步长填充 (%" PRIu32 " > %" PRIu32 ") — 正在剥离",
                     s_fb_stride, s_fb_width);
        } else {
            ESP_LOGE(TAG, "分配步长剥离缓冲区失败");
        }
    }
    /* RGB565: 无需转换 — ISP 输出直接送入 JPEG 编码器 */

    /* 申请缓冲区 */
    struct v4l2_requestbuffers req = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .count = MAX_FB_NUM,
    };
    ESP_RETURN_ON_FALSE(ioctl(s_video_fd, VIDIOC_REQBUFS, &req) == 0, ESP_FAIL, TAG, "REQBUFS 失败");
    s_nbufs = (int)req.count;

    for (int i = 0; i < s_nbufs; i++) {
        struct v4l2_buffer q = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = (uint32_t)i };
        ESP_RETURN_ON_FALSE(ioctl(s_video_fd, VIDIOC_QUERYBUF, &q) == 0, ESP_FAIL, TAG, "QUERYBUF 失败");
        s_mmap_buf[i] = mmap(NULL, q.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_video_fd, q.m.offset);
        ESP_RETURN_ON_FALSE(s_mmap_buf[i] != MAP_FAILED, ESP_FAIL, TAG, "mmap 失败");
    }
    for (int i = 0; i < s_nbufs; i++) {
        struct v4l2_buffer q = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = (uint32_t)i };
        ESP_RETURN_ON_FALSE(ioctl(s_video_fd, VIDIOC_QBUF, &q) == 0, ESP_FAIL, TAG, "QBUF 失败");
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_RETURN_ON_FALSE(ioctl(s_video_fd, VIDIOC_STREAMON, &type) == 0, ESP_FAIL, TAG, "STREAMON 失败");

    ESP_LOGI(TAG, "  [3/3] JPEG 初始化...");
    ESP_RETURN_ON_FALSE(jpeg_init(quality) == ESP_OK, ESP_FAIL, TAG, "JPEG 失败");

    /* 初始化输出缓冲区 */
    s_jpeg_out[0] = cam_alloc(JPEG_BUF_SIZE);
    s_jpeg_out[1] = cam_alloc(JPEG_BUF_SIZE);
    if (!s_jpeg_out[0] || !s_jpeg_out[1]) {
        ESP_LOGW(TAG, "双缓冲区分配失败，将延迟初始化");
    }

    s_frame_mutex = xSemaphoreCreateMutex();
    s_frame_ready = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_frame_mutex && s_frame_ready, ESP_ERR_NO_MEM, TAG, "信号量失败");
    /* s_frame_ready 初始为 0 — 第一帧会发布它 */

    ESP_RETURN_ON_FALSE(
        xTaskCreate(camera_task, "CamV4L2", CAM_TASK_STACK, NULL, CAM_TASK_PRIO, &s_cam_task) == pdPASS,
        ESP_FAIL, TAG, "任务失败");

    /* 传感器推流后初始化自动曝光 */
    vTaskDelay(pdMS_TO_TICKS(50));
    ae_init();

    ESP_LOGI(TAG, "摄像头就绪! %" PRIu32 "x%" PRIu32 " 步长=%" PRIu32,
             s_fb_width, s_fb_height, s_fb_stride);
    return ESP_OK;

fail0:
    esp_video_deinit();
    return ESP_FAIL;
}

esp_err_t camera_module_get_frame(const uint8_t **jpeg_buf, size_t *jpeg_len)
{
    if (!jpeg_buf || !jpeg_len) return ESP_ERR_INVALID_ARG;

    /* 阻塞等待新帧 (摄像头任务每帧发布一次) */
    if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(200)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    int idx = s_published_idx;
    *jpeg_buf = s_jpeg_out[idx];
    *jpeg_len = s_jpeg_out_len[idx];

    xSemaphoreGive(s_frame_mutex);
    return (*jpeg_buf && *jpeg_len) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t camera_module_start(void) { return ESP_OK; }

/* ==================== 公开 API: FPS + AE 状态 ==================== */
uint32_t camera_module_get_fps(void)
{
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - s_fps_last_us;

    /* 约每 500 ms 重新计算，保持数值新鲜 */
    if (elapsed_us > 500000) {
        uint32_t count = s_fps_counter;
        s_fps_counter = 0;
        if (elapsed_us > 0) {
            s_cached_fps = (uint32_t)((count * 1000000ULL) / (uint64_t)elapsed_us);
        }
        s_fps_last_us = now_us;
    }
    return s_cached_fps;
}

void camera_module_get_ae_status(float *brightness, int32_t *exp_100us, int32_t *gain_idx)
{
    if (brightness)  *brightness  = s_ae.smooth_brightness;
    if (exp_100us)   *exp_100us   = (int32_t)s_ae.exp_100us;
    if (gain_idx)    *gain_idx    = (int32_t)s_ae.gain_idx;
}

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
    ESP_LOGI(TAG, "摄像头已停止");
    return ESP_OK;
}

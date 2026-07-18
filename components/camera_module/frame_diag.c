/*
 * Frame diagnostics — samples RGB565 frames for flicker analysis.
 * Called from camera task right after DMA memcpy, before overlay drawing.
 */
#include <string.h>
#include <math.h>
#include "frame_diag.h"

/* ── Config ── */
#define SAMPLE_SKIP  4        /* sample every Nth pixel to keep it fast */
#define BLACK_THRESH  16      /* pixel < this is "black" */
#define BLACK_RATIO   0.90f   /* >this fraction black => is_black frame */
#define COLOR_SHIFT_RATIO 2.5f /* R/G or G/R > this => color shift */
#define JUMP_THRESH   0.30f   /* 30% brightness change => jump */

static frame_diag_ctx_t s_ctx;

void frame_diag_sample(const uint8_t *rgb565, int w, int h, int stride)
{
    frame_diag_t *f = &s_ctx.frames[s_ctx.head];

    memset(f, 0, sizeof(*f));
    f->seq = s_ctx.count;

    /* ── Sample pixels ── */
    int sample_count = 0;
    float sum_r = 0, sum_g = 0, sum_b = 0;
    float sum_r2 = 0, sum_g2 = 0, sum_b2 = 0;
    int black_pixels = 0;
    float g_min = 255.0f, g_max = 0.0f;

    for (int y = 0; y < h; y += SAMPLE_SKIP) {
        const uint16_t *line = (const uint16_t *)(rgb565 + y * stride);
        for (int x = 0; x < w; x += SAMPLE_SKIP) {
            uint16_t p = line[x];
            float r5 = (float)((p >> 11) & 0x1F);
            float g6 = (float)((p >> 5) & 0x3F);
            float b5 = (float)(p & 0x1F);

            /* Expand to 8-bit scale */
            float r8 = r5 * 8.2258f;   /* 255/31 */
            float g8 = g6 * 4.0476f;   /* 255/63 */
            float b8 = b5 * 8.2258f;

            sum_r += r8; sum_g += g8; sum_b += b8;
            sum_r2 += r8*r8; sum_g2 += g8*g8; sum_b2 += b8*b8;

            if (g8 < g_min) g_min = g8;
            if (g8 > g_max) g_max = g8;
            if (g8 < BLACK_THRESH) black_pixels++;

            sample_count++;
        }
    }

    if (sample_count == 0) return;

    float inv_n = 1.0f / (float)sample_count;
    f->r_mean = sum_r * inv_n;
    f->g_mean = sum_g * inv_n;
    f->b_mean = sum_b * inv_n;
    f->r_std  = sqrtf(fmaxf(sum_r2 * inv_n - f->r_mean * f->r_mean, 0));
    f->g_std  = sqrtf(fmaxf(sum_g2 * inv_n - f->g_mean * f->g_mean, 0));
    f->b_std  = sqrtf(fmaxf(sum_b2 * inv_n - f->b_mean * f->b_mean, 0));
    f->g_min  = g_min;
    f->g_max  = g_max;

    /* ── Anomaly detection ── */
    float black_ratio = (float)black_pixels * inv_n;
    f->is_black = (black_ratio > BLACK_RATIO);

    if (f->g_mean > 5.0f) {  /* not a black frame */
        float rg = f->r_mean / f->g_mean;
        float bg = f->b_mean / f->g_mean;
        f->is_green_shift = (rg < 0.3f && bg < 0.3f);
        f->is_red_shift   = (rg > 3.0f);
    }

    /* ── Brightness jump detection ── */
    if (s_ctx.count > 0 && s_ctx.last_g_mean > 5.0f && f->g_mean > 5.0f) {
        float delta = fabsf(f->g_mean - s_ctx.last_g_mean);
        float ref = fmaxf(s_ctx.last_g_mean, 5.0f);
        if (delta / ref > JUMP_THRESH) s_ctx.jump_count++;
    }

    /* ── Update counters ── */
    if (f->is_black)        s_ctx.black_count++;
    if (f->is_green_shift)  s_ctx.green_shift_count++;
    if (f->is_red_shift)    s_ctx.red_shift_count++;
    s_ctx.last_g_mean = f->g_mean;

    /* ── Advance ring buffer ── */
    s_ctx.head = (s_ctx.head + 1) % FRAME_DIAG_MAX;
    s_ctx.count++;
}

const frame_diag_ctx_t *frame_diag_get_ctx(void)
{
    return &s_ctx;
}

void frame_diag_reset(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
}

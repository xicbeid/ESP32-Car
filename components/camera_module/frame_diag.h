#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Per-frame stats (all floats, compact) */
typedef struct {
    uint32_t seq;          /* frame sequence number */
    float    r_mean;       /* red channel mean   [0-255] */
    float    g_mean;       /* green channel mean [0-255] */
    float    b_mean;       /* blue channel mean  [0-255] */
    float    r_std;        /* red channel std dev */
    float    g_std;        /* green channel std dev */
    float    b_std;        /* blue channel std dev */
    float    g_min;        /* min green in sample */
    float    g_max;        /* max green in sample */
    bool     is_black;     /* >90% pixels below 16 */
    bool     is_green_shift; /* G >> R,B (awb oscillation) */
    bool     is_red_shift;   /* R >> G,B (cache stale?) */
} frame_diag_t;

/* Ring buffer capacity */
#define FRAME_DIAG_MAX 256

typedef struct {
    frame_diag_t frames[FRAME_DIAG_MAX];
    uint32_t     head;       /* next write slot */
    uint32_t     count;      /* total frames sampled (wraps at 2^32) */
    uint32_t     black_count;
    uint32_t     green_shift_count;
    uint32_t     red_shift_count;
    uint32_t     jump_count; /* >30% brightness change between consecutive frames */
    float        last_g_mean;
} frame_diag_ctx_t;

/* Sample one frame from an RGB565 buffer.
 * Call from camera task right after memcpy to DMA buf, before draw. */
void frame_diag_sample(const uint8_t *rgb565, int w, int h, int stride);

/* Get the ring buffer context for the HTTP handler to read */
const frame_diag_ctx_t *frame_diag_get_ctx(void);

/* Reset all counters */
void frame_diag_reset(void);

#ifdef __cplusplus
}
#endif

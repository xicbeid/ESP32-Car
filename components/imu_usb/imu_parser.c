/*
 * IMU 解析器 — Wit-Motion 标准协议 (C 实现)
 *
 * 从 Wit-Motion imu_usb.py (V1.5.1) 和 wit_c_sdk.c 移植
 * 新增: EMA 低通滤波器，用于运动平台的振动抑制
 */

#include "imu_parser.h"
#include <string.h>

/* ── 常量 ────────────────────────────────────────────────── */

#define FRAME_LEN       11
#define HEADER_BYTE     0x55

/* 数据类型标识 */
#define TYPE_ACC        0x51
#define TYPE_GYRO       0x52
#define TYPE_ANGLE      0x53
#define TYPE_MAG        0x54
#define TYPE_PORT       0x55
#define TYPE_PRESS      0x56
#define TYPE_GPS        0x57
#define TYPE_VELOCITY   0x58
#define TYPE_QUAT       0x59
#define TYPE_GSA        0x5A
#define TYPE_REG        0x5F

/* 量程缩放因子 */
#define K_ACC           16.0f
#define K_GYRO          2000.0f
#define K_ANGLE         180.0f
#define SCALE_INT16     32768.0f

/* EMA 滤波系数 (0 < alpha <= 1, 越小越平滑) */
#define EMA_ALPHA       0.15f

/* ── 解析器状态 ─────────────────────────────────────────── */

static struct {
    uint8_t  buf[FRAME_LEN];
    uint8_t  idx;          /* buf 写入位置 */
    bool     synced;       /* 已找到 0x55 帧头 */
    imu_data_t raw;        /* 最新原始解析数据 */
    imu_data_t filtered;   /* EMA 滤波后输出 */
    imu_data_ready_cb_t callback;
} s_ctx;

/* ── 辅助函数 ────────────────────────────────────────────── */

static inline int16_t bytes_to_i16(uint8_t lo, uint8_t hi)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

static float scale_i16(int16_t raw, float k)
{
    float val = (float)raw / SCALE_INT16 * k;
    if (val >= k) val -= 2.0f * k;
    return val;
}

static inline float ema(float prev, float cur)
{
    return prev + EMA_ALPHA * (cur - prev);
}

static void reset_frame(void)
{
    s_ctx.idx = 0;
    s_ctx.synced = false;
}

/* ── 帧分发器 ───────────────────────────────────────────── */

static void process_frame(void)
{
    uint8_t type  = s_ctx.buf[1];
    uint8_t cksum = s_ctx.buf[FRAME_LEN - 1];

    /* 校验和: 字节 0..9 累加，取低 8 位 */
    uint8_t sum = 0;
    for (int i = 0; i < FRAME_LEN - 1; i++) {
        sum += s_ctx.buf[i];
    }
    if (sum != cksum) return;

    /* 提取 4 个 int16 值 (低字节在前) */
    int16_t d0 = bytes_to_i16(s_ctx.buf[2], s_ctx.buf[3]);
    int16_t d1 = bytes_to_i16(s_ctx.buf[4], s_ctx.buf[5]);
    int16_t d2 = bytes_to_i16(s_ctx.buf[6], s_ctx.buf[7]);
    int16_t d3 = bytes_to_i16(s_ctx.buf[8], s_ctx.buf[9]);

    uint16_t upd = 0;

    switch (type) {
    case TYPE_ACC:
        s_ctx.raw.acc_x = scale_i16(d0, K_ACC);
        s_ctx.raw.acc_y = scale_i16(d1, K_ACC);
        s_ctx.raw.acc_z = scale_i16(d2, K_ACC);
        s_ctx.raw.temperature = (float)d3 / 100.0f;
        upd = IMU_UPD_ACC;
        break;

    case TYPE_GYRO:
        s_ctx.raw.gyro_x = scale_i16(d0, K_GYRO);
        s_ctx.raw.gyro_y = scale_i16(d1, K_GYRO);
        s_ctx.raw.gyro_z = scale_i16(d2, K_GYRO);
        upd = IMU_UPD_GYRO;
        break;

    case TYPE_ANGLE:
        s_ctx.raw.roll  = scale_i16(d0, K_ANGLE);
        s_ctx.raw.pitch = scale_i16(d1, K_ANGLE);
        s_ctx.raw.yaw   = scale_i16(d2, K_ANGLE);
        upd = IMU_UPD_ANGLE;
        break;

    case TYPE_MAG:
        s_ctx.raw.mag_x = (float)d0;
        s_ctx.raw.mag_y = (float)d1;
        s_ctx.raw.mag_z = (float)d2;
        upd = IMU_UPD_MAG;
        break;

    case TYPE_PRESS:
        s_ctx.raw.pressure = (float)(((uint32_t)d1 << 16) | (uint16_t)d0);
        s_ctx.raw.altitude = (float)(((uint32_t)d3 << 16) | (uint16_t)d2);
        upd = IMU_UPD_PRESS;
        break;

    case TYPE_QUAT:
        s_ctx.raw.quat_q0 = scale_i16(d0, 1.0f);
        s_ctx.raw.quat_q1 = scale_i16(d1, 1.0f);
        s_ctx.raw.quat_q2 = scale_i16(d2, 1.0f);
        s_ctx.raw.quat_q3 = scale_i16(d3, 1.0f);
        upd = IMU_UPD_QUAT;
        break;

    default:
        break;
    }

    if (upd) {
        s_ctx.raw.updated |= upd;

        /* 仅对当前帧类型对应的字段进行 EMA 滤波 */
        switch (type) {
        case TYPE_ACC:
            s_ctx.filtered.acc_x  = ema(s_ctx.filtered.acc_x,  s_ctx.raw.acc_x);
            s_ctx.filtered.acc_y  = ema(s_ctx.filtered.acc_y,  s_ctx.raw.acc_y);
            s_ctx.filtered.acc_z  = s_ctx.raw.acc_z;  /* 重力轴 — 保持原始值 */
            s_ctx.filtered.temperature = s_ctx.raw.temperature;
            break;
        case TYPE_GYRO:
            s_ctx.filtered.gyro_x = ema(s_ctx.filtered.gyro_x, s_ctx.raw.gyro_x);
            s_ctx.filtered.gyro_y = ema(s_ctx.filtered.gyro_y, s_ctx.raw.gyro_y);
            s_ctx.filtered.gyro_z = ema(s_ctx.filtered.gyro_z, s_ctx.raw.gyro_z);
            break;
        case TYPE_ANGLE:
            s_ctx.filtered.roll   = ema(s_ctx.filtered.roll,   s_ctx.raw.roll);
            s_ctx.filtered.pitch  = ema(s_ctx.filtered.pitch,  s_ctx.raw.pitch);
            s_ctx.filtered.yaw    = s_ctx.raw.yaw;  /* 航向角 — 保持原始值 */
            break;
        case TYPE_MAG:
            s_ctx.filtered.mag_x  = s_ctx.raw.mag_x;
            s_ctx.filtered.mag_y  = s_ctx.raw.mag_y;
            s_ctx.filtered.mag_z  = s_ctx.raw.mag_z;
            break;
        case TYPE_PRESS:
            s_ctx.filtered.pressure  = s_ctx.raw.pressure;
            s_ctx.filtered.altitude  = s_ctx.raw.altitude;
            break;
        case TYPE_QUAT:
            s_ctx.filtered.quat_q0 = s_ctx.raw.quat_q0;
            s_ctx.filtered.quat_q1 = s_ctx.raw.quat_q1;
            s_ctx.filtered.quat_q2 = s_ctx.raw.quat_q2;
            s_ctx.filtered.quat_q3 = s_ctx.raw.quat_q3;
            break;
        default:
            break;
        }

        s_ctx.filtered.updated |= upd;

        if (s_ctx.callback) {
            s_ctx.callback(&s_ctx.filtered, type);
        }
    }
}

/* ── 公开 API ────────────────────────────────────────────── */

void imu_parser_init(imu_data_ready_cb_t cb)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.callback = cb;
}

void imu_parser_reset(void)
{
    reset_frame();
}

void imu_parser_feed(uint8_t byte)
{
    if (!s_ctx.synced) {
        if (byte == HEADER_BYTE) {
            s_ctx.synced = true;
            s_ctx.idx = 0;
            s_ctx.buf[s_ctx.idx++] = byte;
        }
        return;
    }

    s_ctx.buf[s_ctx.idx++] = byte;

    if (s_ctx.idx >= FRAME_LEN) {
        process_frame();
        reset_frame();
    }
}

void imu_parser_get_data(imu_data_t *out)
{
    *out = s_ctx.filtered;
    s_ctx.filtered.updated = 0;
}

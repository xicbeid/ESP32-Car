/*
 * IMU 解析器 — Wit-Motion 标准协议 (C 实现)
 *
 * 从 Wit-Motion imu_usb.py (V1.5.1) 和 wit_c_sdk.c 移植
 * 新增: EMA 低通滤波器，用于运动平台的振动抑制
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 解析后的传感器数据结构 ─────────────────────────────── */

typedef struct {
    float acc_x;        /* g  (量程 ±16g) */
    float acc_y;
    float acc_z;
    float gyro_x;       /* °/s (量程 ±2000°/s) */
    float gyro_y;
    float gyro_z;
    float roll;         /* ° (量程 ±180°) */
    float pitch;
    float yaw;
    float mag_x;        /* 原始 ADC 值 */
    float mag_y;
    float mag_z;
    float pressure;     /* Pa */
    float altitude;     /* cm */
    float quat_q0;      /* 四元数 */
    float quat_q1;
    float quat_q2;
    float quat_q3;
    float temperature;  /* °C */

    /* 本周期更新的字段位掩码 */
    uint16_t updated;
} imu_data_t;

/* imu_data_t.updated 位标志 */
#define IMU_UPD_ACC         (1 << 0)
#define IMU_UPD_GYRO        (1 << 1)
#define IMU_UPD_ANGLE       (1 << 2)
#define IMU_UPD_MAG         (1 << 3)
#define IMU_UPD_PRESS       (1 << 4)
#define IMU_UPD_QUAT        (1 << 5)

/* ── 回调类型 ────────────────────────────────────────────── */

typedef void (*imu_data_ready_cb_t)(const imu_data_t *data, uint8_t type);

/* ── API ──────────────────────────────────────────────────── */

void imu_parser_init(imu_data_ready_cb_t cb);
void imu_parser_feed(uint8_t byte);
void imu_parser_get_data(imu_data_t *out);
void imu_parser_reset(void);

#ifdef __cplusplus
}
#endif

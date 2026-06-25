/*
 * IMU Parser — Wit-Motion Normal Protocol
 *
 * Parses 11-byte frames from Wit-Motion 10-axis IMU sensors.
 * Protocol: 0x55 header + type + 4x int16 (low/high) + checksum
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Parsed sensor data structure ─────────────────────────────── */

typedef struct {
    float acc_x;        /* g  (range ±16g) */
    float acc_y;
    float acc_z;
    float gyro_x;       /* °/s (range ±2000°/s) */
    float gyro_y;
    float gyro_z;
    float roll;         /* ° (range ±180°) */
    float pitch;
    float yaw;
    float mag_x;        /* raw ADC */
    float mag_y;
    float mag_z;
    float pressure;     /* Pa */
    float altitude;     /* cm */
    float quat_q0;      /* quaternion */
    float quat_q1;
    float quat_q2;
    float quat_q3;
    float temperature;  /* °C */

    /* Bitmask of which fields have been updated this cycle */
    uint16_t updated;
} imu_data_t;

/* Bit flags for imu_data_t.updated */
#define IMU_UPD_ACC         (1 << 0)
#define IMU_UPD_GYRO        (1 << 1)
#define IMU_UPD_ANGLE       (1 << 2)
#define IMU_UPD_MAG         (1 << 3)
#define IMU_UPD_PRESS       (1 << 4)
#define IMU_UPD_QUAT        (1 << 5)

/* ── Callback type ────────────────────────────────────────────── */

typedef void (*imu_data_ready_cb_t)(const imu_data_t *data, uint8_t type);

/* ── API ──────────────────────────────────────────────────────── */

void imu_parser_init(imu_data_ready_cb_t cb);
void imu_parser_feed(uint8_t byte);
void imu_parser_get_data(imu_data_t *out);
void imu_parser_reset(void);

#ifdef __cplusplus
}
#endif

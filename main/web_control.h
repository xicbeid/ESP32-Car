/*
 * Web Control Module — HTTP server + embedded HTML control page
 * Simplified version for UART motor project (no camera, no auto_track)
 */

#ifndef __WEB_CONTROL_H__
#define __WEB_CONTROL_H__

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Motor command types */
typedef enum {
    MOTOR_CMD_FORWARD = 0,
    MOTOR_CMD_BACKWARD,
    MOTOR_CMD_LEFT,
    MOTOR_CMD_RIGHT,
    MOTOR_CMD_STOP,
} motor_cmd_t;

typedef void (*motor_control_cb_t)(motor_cmd_t cmd, int distance_cm, int speed);

/**
 * @brief Start the web remote-control HTTP server.
 * @param callback  Called when a motor command is received.
 */
esp_err_t web_control_start(motor_control_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif

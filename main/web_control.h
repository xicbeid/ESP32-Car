/*
 * Web Control Module — HTTP server + embedded HTML control page
 *
 * Port 80 (httpd): page, motor ctrl, snapshot, favicon
 * Port 81 (raw TCP): MJPEG camera stream (dedicated task, non-blocking)
 */

#ifndef __WEB_CONTROL_H__
#define __WEB_CONTROL_H__

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Motor command types */
typedef enum {
    MOTOR_CMD_FORWARD = 0,   /* encoder closed-loop forward  (legacy) */
    MOTOR_CMD_BACKWARD,      /* encoder closed-loop backward (legacy) */
    MOTOR_CMD_LEFT,          /* encoder closed-loop left      (legacy) */
    MOTOR_CMD_RIGHT,         /* encoder closed-loop right     (legacy) */
    MOTOR_CMD_STOP,          /* immediate stop + cancel pending move */
    MOTOR_CMD_VEL_FWD,       /* velocity mode: forward  (press/hold → release=stop) */
    MOTOR_CMD_VEL_BACK,      /* velocity mode: backward */
    MOTOR_CMD_VEL_LEFT,      /* velocity mode: turn left */
    MOTOR_CMD_VEL_RIGHT,     /* velocity mode: turn right */
    MOTOR_CMD_GO,            /* encoder closed-loop: go forward <dist> cm at <speed> */
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

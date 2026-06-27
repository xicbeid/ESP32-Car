/*
 * 网页控制模块 — HTTP 服务器 + 嵌入式 HTML 控制页
 *
 * Port 80 (httpd): 页面, 电机控制, 快照, 图标
 * Port 81 (raw TCP): MJPEG 摄像头推流 (专用任务, 非阻塞)
 */

#ifndef __WEB_CONTROL_H__
#define __WEB_CONTROL_H__

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 电机命令类型 */
typedef enum {
    MOTOR_CMD_FORWARD = 0,   /* 编码器闭环前进  (旧接口) */
    MOTOR_CMD_BACKWARD,      /* 编码器闭环后退  (旧接口) */
    MOTOR_CMD_LEFT,          /* 编码器闭环左转  (旧接口) */
    MOTOR_CMD_RIGHT,         /* 编码器闭环右转  (旧接口) */
    MOTOR_CMD_STOP,          /* 立即停车 + 取消待执行移动 */
    MOTOR_CMD_VEL_FWD,       /* 速度模式: 前进  (按住移动 → 松手即停) */
    MOTOR_CMD_VEL_BACK,      /* 速度模式: 后退 */
    MOTOR_CMD_VEL_LEFT,      /* 速度模式: 原地左转 */
    MOTOR_CMD_VEL_RIGHT,     /* 速度模式: 原地右转 */
    MOTOR_CMD_GO,            /* 编码器闭环: 前进 <dist> cm, 速度 <speed> */
} motor_cmd_t;

typedef void (*motor_control_cb_t)(motor_cmd_t cmd, int distance_cm, int speed);

/**
 * @brief 启动网页遥控 HTTP 服务器。
 * @param callback  收到电机命令时回调。
 */
esp_err_t web_control_start(motor_control_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif

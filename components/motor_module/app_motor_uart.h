#ifndef __APP_MOTOR_USART_H_
#define __APP_MOTOR_USART_H_

#include "uart_module.h"
#include <string.h>

typedef unsigned char u8;

/* 外部声明区 */

/* 电机类型枚举，用于判定死区 */
typedef enum _motor_type
{
    MOTOR_TYPE_NONE = 0x00,       /* 保留 */
    MOTOR_520 ,       /* 520 电机 */
    MOTOR_310 ,       /* 310 电机 */
    MOTOR_TT_Encoder ,  /* TT 电机，带编码器 */
    MOTOR_TT ,        /* TT 电机，不带编码器 */

    Motor_TYPE_MAX                /* 最后一个电机类型，仅用于判断 */
} motor_type_t;

/* 导出编码器变量供外部使用 */
extern int Encoder_Offset[4];
extern int Encoder_Now[4];
extern float g_Speed[4];
extern uint8_t g_recv_flag;

void send_motor_type(motor_type_t data);
void send_motor_deadzone(uint16_t data);
void send_pulse_line(uint16_t data);
void send_pulse_phase(uint16_t data);
void send_wheel_diameter(float data);
void send_motor_PID(float P,float I,float D);
void send_upload_data(bool ALLEncoder_Switch,bool TenEncoder_Switch,bool Speed_Switch);
void Contrl_Speed(int16_t M1_speed,int16_t M2_speed,int16_t M3_speed,int16_t M4_speed);
void Contrl_Pwm(int16_t M1_pwm,int16_t M2_pwm,int16_t M3_pwm,int16_t M4_pwm);

void Deal_Control_Rxtemp(uint8_t rxtemp);
void Deal_data_real(void);

#endif

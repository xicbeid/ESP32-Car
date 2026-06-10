#ifndef __UART_MODULE_H__
#define __UART_MODULE_H__

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

// 引脚配置 (ESP32-P4 UART0) Pin Configuration
#define UART0_TX_PIN    20
#define UART0_RX_PIN    21
#define UART_BAUD  115200

extern QueueHandle_t uart_queue;

void uart0_init(void);
int Send_Motor_ArrayU8(uint8_t* data, uint16_t len);
int Send_Motor_U8(uint8_t data);
void UART_Process_Task(void *pvParameters);


#endif
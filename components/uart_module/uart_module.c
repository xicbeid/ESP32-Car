#include "uart_module.h"
#include "app_motor_uart.h"

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

QueueHandle_t uart_queue;

int Send_Motor_ArrayU8(uint8_t* data, uint16_t len)
{
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    return txBytes;
}

int Send_Motor_U8(uint8_t data)
{
    uint8_t data1 = data;
    const int txBytes = uart_write_bytes(UART_NUM_0, &data1, 1);
    return txBytes;
}

void uart0_init()
{
    /* If console driver already owns UART0, tear it down first.
     * Otherwise our RX event queue won't be created and
     * UART_Process_Task will crash on xQueueReceive(NULL). */
    if (uart_is_driver_installed(UART_NUM_0)) {
        ESP_LOGW("UART", "UART0 driver already installed (console) — re-initializing for motor");
        uart_driver_delete(UART_NUM_0);
    }

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_RTC,
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART0_TX_PIN, UART0_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    /* Install driver with RX event queue — motor UART_Process_Task needs this */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2048, 2048, 50, &uart_queue, 0));
}

void UART_Process_Task(void *pvParameters) {
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    if (!dtmp) { ESP_LOGE("UART", "RX buf alloc fail"); vTaskDelete(NULL); return; }

    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    int len = uart_read_bytes(UART_NUM_0, dtmp, event.size, portMAX_DELAY);
                    for (int i = 0; i < len; i++) {
                        Deal_Control_Rxtemp(dtmp[i]);
                    }
                    break;
                case UART_BUFFER_FULL:
                    uart_flush_input(UART_NUM_0);
                    xQueueReset(uart_queue);
                    break;
                case UART_FIFO_OVF:
                    uart_flush_input(UART_NUM_0);
                    xQueueReset(uart_queue);
                    break;
                default:
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

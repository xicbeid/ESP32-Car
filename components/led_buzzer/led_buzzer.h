/*
 * LED + Buzzer GPIO control (external power, active-low)
 *
 * LED   GPIO23: ON  = WiFi AP up, no clients
 *               OFF = client connected
 * Buzzer GPIO22: "beep-beep" pattern when face found, silent otherwise
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Init GPIOs. Call after NVS. */
esp_err_t led_buzzer_init(void);

/** Start buzzer pattern task. Call once after init. */
void led_buzzer_start_tasks(void);

/** LED state. Active-low. */
void led_set(bool on);
void led_wifi_ready(void);
void led_wifi_client_joined(void);
void led_wifi_all_left(void);

/** Buzzer: enable/disable "beep-beep" pattern. Active-low. */
void buzzer_set(bool on);

/** Light (relay): GPIO26, active-low. on=true → lamp ON. */
void light_init(void);
void light_set(bool on);
bool light_is_on(void);

#ifdef __cplusplus
}
#endif

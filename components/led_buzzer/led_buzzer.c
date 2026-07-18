/*
 * LED (GPIO23) + Buzzer (GPIO22) — active-low GPIOs, external power.
 *
 * LED:   steady ON/OFF based on WiFi client status
 * Buzzer: "beep-beep" pattern when face detected
 *         ├─ ON  100ms ─┤├─ OFF 100ms ─┤├─ ON 100ms ─┤├─ OFF 700ms ─┤ repeat
 */
#include "led_buzzer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "LED-BZR"
#define LED_PIN    GPIO_NUM_23
#define BUZZER_PIN GPIO_NUM_22
#define LIGHT_PIN  GPIO_NUM_26   /* 继电器, 低电平=亮, 高电平=灭 */

#define delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))

static volatile bool s_buzzer_active = false;

esp_err_t led_buzzer_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_PIN) | (1ULL << BUZZER_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    /* Active-low: high = both OFF */
    gpio_set_level(LED_PIN, 1);
    gpio_set_level(BUZZER_PIN, 1);

    ESP_LOGI(TAG, "LED GPIO%d Buzzer GPIO%d ready", LED_PIN, BUZZER_PIN);
    return ESP_OK;
}

/* ── Buzzer pattern task ── */
static void buzzer_task(void *arg)
{
    while (1) {
        if (!s_buzzer_active) {
            gpio_set_level(BUZZER_PIN, 1);  /* off */
            delay_ms(50);                    /* poll every 50ms */
            continue;
        }

        /* "滴滴" = ON 100ms, OFF 100ms, ON 100ms */
        gpio_set_level(BUZZER_PIN, 0);  delay_ms(100);  /* di */
        gpio_set_level(BUZZER_PIN, 1);  delay_ms(100);  /* (gap) */
        gpio_set_level(BUZZER_PIN, 0);  delay_ms(100);  /* di */
        gpio_set_level(BUZZER_PIN, 1);  delay_ms(700);  /* pause between patterns */
    }
}

void buzzer_set(bool on)
{
    if (on != s_buzzer_active) {
        s_buzzer_active = on;
        if (!on) gpio_set_level(BUZZER_PIN, 1);  /* immediately silence */
        if (on)  ESP_LOGD(TAG, "Buzzer pattern start");
        else     ESP_LOGD(TAG, "Buzzer stop");
    }
}

/* ── LED ── */

void led_set(bool on)
{
    gpio_set_level(LED_PIN, on ? 0 : 1);  /* active-low */
}

void led_wifi_ready(void)
{
    ESP_LOGI(TAG, "LED ON — WiFi ready");
    led_set(true);
}

void led_wifi_client_joined(void)
{
    ESP_LOGI(TAG, "LED OFF — client connected");
    led_set(false);
}

void led_wifi_all_left(void)
{
    ESP_LOGI(TAG, "LED ON — all clients left");
    led_set(true);
}

/* Call once after led_buzzer_init, starts the pattern task */
void led_buzzer_start_tasks(void)
{
    xTaskCreate(buzzer_task, "BuzzerTask", 2048, NULL, 1, NULL);
}

/* ── Light (relay) ── */

static bool s_light_on = false;

void light_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LIGHT_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(LIGHT_PIN, 1);  /* 默认灭灯 */
    ESP_LOGI(TAG, "Light relay GPIO%d init (OFF)", LIGHT_PIN);
}

void light_set(bool on)
{
    s_light_on = on;
    gpio_set_level(LIGHT_PIN, on ? 0 : 1);  /* 低电平=亮 */
    ESP_LOGI(TAG, "Light: %s", on ? "ON" : "OFF");
}

bool light_is_on(void)
{
    return s_light_on;
}

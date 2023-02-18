#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define LED_PIN GPIO_NUM_2
#define TICK_INTERVAL_US (100000)

void led_toggle(void);

void app_main()
{
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // Configurar el temporizador
    esp_timer_handle_t timer_handle;
    esp_timer_create_args_t timer_args = {
        .callback = led_toggle,
        .dispatch_method = ESP_TIMER_TASK
    };
    esp_timer_create(&timer_args, &timer_handle);
    esp_timer_start_periodic(timer_handle, TICK_INTERVAL_US);
}

void led_toggle(void)
{
    static int state = 0;
    state = !state;
    gpio_set_level(LED_PIN, state);
}
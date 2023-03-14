#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define LED_PIN 2
#define BUTTON_PIN 4


void init_ledc();
void init_ledc_channel(int gpio_num, int channel_num);
void led_fade(int channel_num, int target_duty);
void led_task(void* pvParameters);
void init_button(int gpio_num);
void button_task(void* pvParameters);


void app_main()
{
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
}


void init_ledc()
{
    ledc_timer_config_t timer_conf;
    timer_conf.duty_resolution = LEDC_TIMER_10_BIT;
    timer_conf.freq_hz = 5000;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = LEDC_TIMER_0;
    ledc_timer_config(&timer_conf);
}
void init_ledc_channel(int gpio_num, int channel_num)
{
    ledc_channel_config_t channel_conf;
    channel_conf.channel = channel_num;
    channel_conf.duty = 0;
    channel_conf.gpio_num = gpio_num;
    channel_conf.intr_type = LEDC_INTR_DISABLE;
    channel_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    channel_conf.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&channel_conf);
}
void led_fade(int channel_num, int target_duty)
{
    int current_duty = ledc_get_duty(LEDC_HIGH_SPEED_MODE, channel_num);
    while (current_duty != target_duty) {
        if (target_duty > current_duty) {
            current_duty++;
        } else {
            current_duty--;
        }
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel_num, current_duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel_num);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void led_task(void* pvParameters)
{
    init_ledc();
    init_ledc_channel(LED_PIN, LEDC_CHANNEL_0);
    while (true) {
        led_fade(LEDC_CHANNEL_0, 1023);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        led_fade(LEDC_CHANNEL_0, 0);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
void init_button(int gpio_num)
{
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio_num, GPIO_PULLUP_ONLY);
}
void button_task(void* pvParameters)
{
    init_button(BUTTON_PIN);
    bool button_pressed = false;
    while (true) {
        if (gpio_get_level(BUTTON_PIN) == 0 && !button_pressed) {  // Pulsador presionado
            button_pressed = true;
            led_fade(LEDC_CHANNEL_0, 1023);  // Comenzar el fade
        } else if (gpio_get_level(BUTTON_PIN) == 1 && button_pressed) {  // Pulsador liberado
            button_pressed = false;
            led_fade(LEDC_CHANNEL_0, 0);  // Detener el fade
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Pequeño delay para evitar la saturación de la CPU
    }
}
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_intr_alloc.h"
#include "driver/uart.h"
#include "freertos/queue.h"


#define LEDR_PIN GPIO_NUM_27
#define LEDG_PIN GPIO_NUM_20
#define LEDB_PIN GPIO_NUM_21
#define BUTTON_PIN GPIO_NUM_0
#define ANALOG_PIN ADC1_CHANNEL_6

#define UART_TX_PIN GPIO_NUM_1
#define UART_RX_PIN GPIO_NUM_3

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)


static QueueHandle_t uart0_queue;

static void gpio_init()
{
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LEDR_PIN | 1ULL << LEDG_PIN | 1ULL << LEDB_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_config);

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&button_config);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ANALOG_PIN, ADC_ATTEN_DB_11);
}

static void uart_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    while (1) {
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            if (event.type == UART_DATA) {
                uint8_t data[8];
                int len = uart_read_bytes(UART_NUM_0, data, sizeof(data), portMAX_DELAY);

                if (len > 0) {
                    if (strncmp((char *)data, "LEDR_ON", len) == 0) {
                        gpio_set_level(LEDR_PIN, 1);
                    } else if (strncmp((char *)data, "LEDR_OFF", len) == 0) {
                        gpio_set_level(LEDR_PIN, 0);
                    } else if (strncmp((char *)data, "LEDG_ON", len) == 0) {
                        gpio_set_level(LEDG_PIN, 1);
                    } else if (strncmp((char *)data, "LEDG_OFF", len) == 0) {
                        gpio_set_level(LEDG_PIN, 0);
                    } else if (strncmp((char *)data, "LEDB_ON", len) == 0) {
                        gpio_set_level(LEDB_PIN, 1);
                    } else if (strncmp((char *)data, "LEDB_OFF", len) == 0) {
                        gpio_set_level(LEDB_PIN, 0);
                    }
                }
            }
        }

        vTaskDelay(NULL);
    }
}

static void IRAM_ATTR button_isr_handler(void* arg)
{
    uint32_t adc_reading = adc1_get_raw(ANALOG_PIN);
    char buffer[16];
    int8_t len = sprintf(buffer, sizeof(buffer), "%d", adc_reading);
    uart_write_bytes(UART_NUM_0, buffer, len);
}

static void button_init()
{
    gpio_install_isr_service(0);

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&button_config);

    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);
    gpio_intr_enable(BUTTON_PIN);
}

void app_main()
{
    gpio_init();

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL, 0);

    xTaskCreate(uart_task, "uart_task", 2048, NULL, 10, NULL);

    button_init();
}


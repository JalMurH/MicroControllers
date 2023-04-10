#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
#include <math.h>

#define LED_RED_GPIO   25
#define LED_GREEN_GPIO 27
#define LED_BLUE_GPIO  26
#define BLINK_LED_GPIO 2    

#define BTN_UP_GPIO    32
#define BTN_DOWN_GPIO  33

#define ADC_CHANNEL    ADC1_CHANNEL_0

//Create the Queues
QueueHandle_t uart_queue;
QueueHandle_t led_queue;

//Create struct of params to UARTTask and LEDTask
typedef struct {
  float tmin_r;
  float tmax_r;
  float tmin_g;
  float tmax_g;
  float tmin_b;
  float tmax_b;
  float temperature;
  float threshold;
} TemperatureParams;


static void gpio_config_task(void *pvParameters) {

    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << LED_RED_GPIO) | (1ULL << LED_GREEN_GPIO) | (1ULL << LED_BLUE_GPIO) | (1ULL << BLINK_LED_GPIO);
    gpio_config(&io_conf);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BTN_UP_GPIO) | (1ULL << BTN_DOWN_GPIO);
    gpio_config(&io_conf);

    vTaskDelete(NULL);
}

void adc_task(void *pvParameters) {

    TemperatureParams params;

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
    while (1) {
        int adc_reading = adc1_get_raw(ADC_CHANNEL);
        float voltage = (float)adc_reading / 4095.0 * 3.3; // Conversión a voltaje
        float resistance = (3300.0 - voltage * 1000.0) / (voltage); // Cálculo de la resistencia del termistor
        params.temperature = (1.0 / (1.0 / 298.15 + 1.0 / 3434.0 * log(resistance / 10000.0))) - 273.15; // Cálculo de la temperatura del termistor
        printf("Temperatura: %.2f°C\n",params.temperature);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Esperar un segundo antes de leer el ADC de nuevo
    }
}

static void uart_task(void *pvParameters) {
    //
    TemperatureParams params;

    char uart_cmd[32];
    int len;

    // Configurar el puerto UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);

    uart_write_bytes(UART_NUM_0, "Comando no reconocido\r\n", 23);

    while (1) {
        // Leer comando UART del usuario
       len = uart_read_bytes(UART_NUM_0, (uint8_t*)uart_cmd, sizeof(uart_cmd), 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            uart_cmd[len] = '\0';
            printf("Comando recibido: %s\n", uart_cmd);
        // Analizar el comando y realizar las acciones correspondientes
        if (strcmp(uart_cmd, "TMINR") == 0) {
                len = uart_read_bytes(UART_NUM_0, (uint8_t*)uart_cmd, sizeof(uart_cmd), 100 / portTICK_PERIOD_MS);
                if (len > 0) {
                    uart_cmd[len] = '\0';
                    params.tmin_r = atof(uart_cmd);

                    printf("Temperatura mínima para LED rojo definida en %f°C\n", params.tmin_r);
                }
            } else if (strcmp(uart_cmd, "TMAXR") == 0) {
                len = uart_read_bytes(UART_NUM_0, (uint8_t*)uart_cmd, sizeof(uart_cmd), 100 / portTICK_PERIOD_MS);
                if (len > 0) {
                    uart_cmd[len] = '\0';
                    params.tmax_r = atof(uart_cmd);
                    printf("Temperatura máxima para LED rojo definida en %f°C\n", params.tmax_r);
                }
            } else if (strcmp(uart_cmd, "TMING") == 0) {
                len = uart_read_bytes(UART_NUM_0, (uint8_t*)uart_cmd, sizeof(uart_cmd), 100 / portTICK_PERIOD_MS);
                if (len > 0) {
                    uart_cmd[len] = '\0';
                    params.tmin_g = atof(uart_cmd);
                    printf("Temperatura mínima para LED Verde definida en %f°C\n", params.tmin_g);
                }
            } else if (strcmp(uart_cmd, "TMAXG") == 0) {
                len = uart_read_bytes(UART_NUM_0, (uint8_t*)uart_cmd, sizeof(uart_cmd), 100 / portTICK_PERIOD_MS);
                if (len > 0) {
                    uart_cmd[len] = '\0';
                    params.tmax_g = atof(uart_cmd);
                    printf("Temperatura máxima para LED Verde definida en %f°C\n", params.tmax_g);
                }
            } else if (strcmp(uart_cmd, "TMINB") == 0) {
                len = uart_read_bytes(UART_NUM_0, (uint8_t*)uart_cmd, sizeof(uart_cmd), 100 / portTICK_PERIOD_MS);
                if (len > 0) {
                    uart_cmd[len] = '\0';
                    params.tmin_b = atof(uart_cmd);
                    printf("Temperatura mínima para LED Azul definida en %f°C\n", params.tmin_b);
                }
            } else if (strcmp(uart_cmd, "TMAXB") == 0) {
                len = uart_read_bytes(UART_NUM_0, (uint8_t*)uart_cmd, sizeof(uart_cmd), 100 / portTICK_PERIOD_MS);
                if (len > 0) {
                    uart_cmd[len] = '\0';
                    params.tmax_b = atof(uart_cmd);
                    printf("Temperatura máxima para LED Azul definida en %f°C\n", params.tmax_b);
                }
            } else if (strcmp(uart_cmd, "THRESHOLD") == 0) {
                len = uart_read_bytes(UART_NUM_0, (uint8_t*)uart_cmd, sizeof(uart_cmd), 100 / portTICK_PERIOD_MS);
                if (len > 0) {
                    uart_cmd[len] = '\0';
                    params.threshold = atof(uart_cmd);
                    printf("Threshold para LED Blink definida en %f°C\n", params.threshold);
                }
            }else {
                // Comando no reconocido
                uart_write_bytes(UART_NUM_0, "Comando no reconocido\r\n", 23);
            }
            //Send modified structure values
            xQueueSend(led_queue, &params, portMAX_DELAY);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}
static void blink_task(void *pvParameters) {
    int blink_interval = 1000;
    int state = 0;
    while (1) {
        state = !state;
        gpio_set_level(BLINK_LED_GPIO, state);
        vTaskDelay(blink_interval / portTICK_PERIOD_MS);
    }
}

static void led_task(void *pvParameters) {
    TemperatureParams params;
    while (1) {
        if (xQueueReceive(led_queue, &params, portMAX_DELAY) == pdPASS) {
            // Leer la variable global de temperatura
            if (params.temperature <= params.tmax_r && params.temperature >= params.tmin_r) {
                // Encender el LED rojo
                gpio_set_level(LED_RED_GPIO, 1); // Encender LED rojo
                gpio_set_level(LED_GREEN_GPIO, 0); // Apagar LED verde
                gpio_set_level(LED_BLUE_GPIO, 0); // Apagar LED azul
            } else if (params.temperature <= params.tmax_g && params.temperature >= params.tmin_g ) {
                // Encender el LED verde
                gpio_set_level(LED_RED_GPIO, 0); // Apagar LED rojo
                gpio_set_level(LED_GREEN_GPIO, 1); // Encender LED verde
                gpio_set_level(LED_BLUE_GPIO, 0); // Apagar LED azul
            } else if (params.temperature <= params.tmax_b && params.temperature >= params.tmin_b){
                gpio_set_level(LED_RED_GPIO, 0); // Apagar LED rojo
                gpio_set_level(LED_GREEN_GPIO, 0); // Apagar LED verde
                gpio_set_level(LED_BLUE_GPIO, 1); // Encender LED azul
            }else if (params.temperature > params.threshold){
                //User threshold overpass
                xTaskCreate(blink_task, "blinktask", 2048, NULL , 2, NULL);
            }else {
                gpio_set_level(LED_RED_GPIO, 0); // Apagar LED rojo
                gpio_set_level(LED_GREEN_GPIO, 0); // Apagar LED verde
                gpio_set_level(LED_BLUE_GPIO, 0); // Apagar LED azul
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}




void app_main(void)
{
    //create queues
    uart_queue = xQueueCreate(10, sizeof(TemperatureParams));
    led_queue = xQueueCreate(10, sizeof(TemperatureParams));

    // Crear tarea de configuración de GPIO
    xTaskCreate(gpio_config_task, "GPIO config", 4096, NULL, 5, NULL);

    // Crear tarea de lectura de ADC
    xTaskCreate(adc_task, "ADC read", 4096, NULL, 5, NULL);

    // Crear tarea de lectura de comandos UART
    xTaskCreate(uart_task, "UART read", 4096, NULL, 5, NULL);
    
    // Enciende y apaga los led segun los estados de las variables
    xTaskCreate(led_task, "Led Set",4096, NULL , 5, NULL);

    vTaskDelay(10);

}
// Mensaje de bienvenida con los comandos posibles
    // printf("Bienvenido! Comandos posibles:\n");
    // printf("  - TMINR: definir temperatura mínima para LED rojo\n");
    // printf("  - TMAXR: definir temperatura máxima para LED rojo\n");
    // printf("  - TMING: definir temperatura mínima para LED verde\n");
    // printf("  - TMAXG: definir temperatura máxima para LED verde\n");
    // printf("  - TMINB: definir temperatura mínima para LED azul\n");
    // printf("  - TMAXB: definir temperatura máxima para LED azul\n");
    // printf("  - THRESHOLD: definir threshold para blink LED\n");

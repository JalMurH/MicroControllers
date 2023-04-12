//LIBRERIAS
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/timers.h"
#include "driver/adc.h"
#include "freertos/queue.h"
#include <math.h>

//MACROS
#define UART_NUM                    UART_NUM_0  //UART 0
#define BUF_SIZE                    1024 * 2    //CAPACIDAD DEL BUFFER

#define BotonSUM                    32          //BOTON SUMA GPIO 32
#define BotonRES                    33          //BOTON RESTA GPIO 33

#define led0                        2           //LED DE LA PLACA GPIO 2
#define ledR                        25          //LED ROJO GPIO 25
#define ledB                        26          //LED AZUL GPIO 26
#define ledG                        27          //LED VERDE GPIO 27

#define Res0                        1000        //VALOR DEL TERMISTOR A 25°C
#define Beta                        3300        //PARAMETRO BETA DEL TERMISTOR
#define Bits_ADC                    4095        //NUMERO DE BITS DEL ADC

//DECLARACION DE VARIABLES A USAR
static const char *tag = "Main";
int interval = 10;
int timerId = 1;
int bandera = 0;
int banderaSUM = 0;
int banderaRES = 0;
int confirmMR = 0;
int confirmmR = 0;
int confirmMG = 0;
int confirmmG = 0;
int confirmMB = 0;
int confirmmB = 0;
int confirmTH = 0;
int num;
int rang_max_R = 60;
int rang_min_R = 45;
int rang_max_B = 50;
int rang_min_B = 35;
int rang_max_G = 40;
int rang_min_G = 30;
int threshold_temp = 20;
int threshold_freq = 1;
float raw = 0;
float Res1 = 1.0;
float temp = 1.0;
float ValADC;
char message_to_send[100];

uint32_t ctimer = 0;
uint16_t ctimers = 0;
uint8_t len;

TimerHandle_t xTimers;

//DECLARACION FUNCIONES
esp_err_t init_led(void);
esp_err_t init_timer(void);
esp_err_t init_ADC(void);
esp_err_t init_isr(void);
void isr_handlerSUM(void *args);
void isr_handlerRES(void *args);

static QueueHandle_t uart_queue; 

static void init_uart(void){                    //FUNCION DE INICIALIZACION DEL UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM,1,3,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM,BUF_SIZE,BUF_SIZE,10,&uart_queue,0);
}

void vTimerCallback(TimerHandle_t pxTimer)      //CALLBACK DEL TIMER
{
    bandera++;
    ctimer++;
    if (ctimer > 1000000 - 1)
    {
        ctimer = 0;
    }
    if (temp > threshold_temp)
        {
            switch (threshold_freq){
            case 1:
                if (ctimer % (100/threshold_freq) == 0)
                {
                    ctimers++;
                    if (ctimers > 10000 - 1)
                    {
                        ctimers = 0;
                    }
                }
                if (ctimers % 2 == 0)
                {
                    gpio_set_level(led0, 0);
                }
                else
                {
                    gpio_set_level(led0, 1);
                }
                break;
            case 2:
                if (ctimer % (100/threshold_freq) == 0)
                {
                    ctimers++;
                    if (ctimers > 10000 - 1)
                    {
                        ctimers = 0;
                    }
                }
                if (ctimers % 2 == 0)
                {
                    gpio_set_level(led0, 0);
                }
                else
                {
                    gpio_set_level(led0, 1);
                }
                break;
            case 3:
                if (ctimer % (100/threshold_freq) == 0)
                {
                    ctimers++;
                    if (ctimers > 10000 - 1)
                    {
                        ctimers = 0;
                    }
                }
                if (ctimers % 2 == 0)
                {
                    gpio_set_level(led0, 0);
                }
                else
                {
                    gpio_set_level(led0, 1);
                }
                break;
            case 4:
                if (ctimer % (100/threshold_freq) == 0)
                {
                    ctimers++;
                    if (ctimers > 10000 - 1)
                    {
                        ctimers = 0;
                    }
                }
                if (ctimers % 2 == 0)
                {
                    gpio_set_level(led0, 0);
                }
                else
                {
                    gpio_set_level(led0, 1);
                }
                break;
            case 5:
                if (ctimer % (100/threshold_freq) == 0)
                {
                    ctimers++;
                    if (ctimers > 10000 - 1)
                    {
                        ctimers = 0;
                    }
                }
                if (ctimers % 2 == 0)
                {
                    gpio_set_level(led0, 0);
                }
                else
                {
                    gpio_set_level(led0, 1);
                }
                break;
            case 6:
                if (ctimer % (100/threshold_freq) == 0)
                {
                    ctimers++;
                    if (ctimers > 10000 - 1)
                    {
                        ctimers = 0;
                    }
                }
                if (ctimers % 2 == 0)
                {
                    gpio_set_level(led0, 0);
                }
                else
                {
                    gpio_set_level(led0, 1);
                }
                break;
            case 7:
                if (ctimer % (100/threshold_freq) == 0)
                {
                    ctimers++;
                    if (ctimers > 10000 - 1)
                    {
                        ctimers = 0;
                    }
                }
                if (ctimers % 2 == 0)
                {
                    gpio_set_level(led0, 0);
                }
                else
                {
                    gpio_set_level(led0, 1);
                }
                break;
            case 8:
                if (ctimer % (100/threshold_freq) == 0)
                {
                    ctimers++;
                    if (ctimers > 10000 - 1)
                    {
                        ctimers = 0;
                    }
                }
                if (ctimers % 2 == 0)
                {
                    gpio_set_level(led0, 0);
                }
                else
                {
                    gpio_set_level(led0, 1);
                }
                break;
            case 9:
                if (ctimer % (100/threshold_freq) == 0)
                {
                    ctimers++;
                    if (ctimers > 10000 - 1)
                    {
                        ctimers = 0;
                    }
                }
                if (ctimers % 2 == 0)
                {
                    gpio_set_level(led0, 0);
                }
                else
                {
                    gpio_set_level(led0, 1);
                }
                break;
            
            case 10:
                if (ctimer % (100/threshold_freq) == 0)
                {
                    ctimers++;
                    if (ctimers > 10000 - 1)
                    {
                        ctimers = 0;
                    }
                }
                if (ctimers % 2 == 0)
                {
                    gpio_set_level(led0, 0);
                }
                else
                {
                    gpio_set_level(led0, 1);
                }
                break;
            default:
                break;
            }
        }else{
            gpio_set_level(led0, 0);
        }
}

void app_main()
{
    init_timer();
    init_led();
    
    init_isr();
    init_ADC();

    init_uart();
    char data[BUF_SIZE];
    uart_event_t event;
    
    //COMANDOS:
    char* comando_get_rangos = "#GET_RANG_TEMP";
    char* comando_get_thresh = "#GET_TRESH_TEMP";
    char* comando_get_temp = "#GET_TEMP";
    char change_thresh[12] = "#CGTRESH$%%$";
    char comparaTMR[12] = "#TEMPTMR$%%$";
    char comparaTmR[12] = "#TEMPTmR$%%$";
    char comparaTMG[12] = "#TEMPTMG$%%$";
    char comparaTmG[12] = "#TEMPTmG$%%$";
    char comparaTMB[12] = "#TEMPTMB$%%$";
    char comparaTmB[12] = "#TEMPTmB$%%$";
    while (1)
    {
        raw = adc1_get_raw(ADC1_CHANNEL_0);     //CAPTURA DEL ADC
        ValADC = raw / Bits_ADC;                //VALOR DEL ADC
        Res1 = ValADC * Res0 / (1 - ValADC);    //VALOR DEL TERMISTOR SEGUN VARIA LA TEMPERATURA
        temp = 1.0 / (1.0 / 298.15 + 1.0 / Beta *(log(Res1 / Res0))) - 273.15;  //CALCULO DE LA TEMPERATURA
        
        //PRINTF DE TESTEO DE VARIABLES
        /*printf("raw %f \n",raw);
        printf("adc %f \n",ValADC);
        printf("res1 %f \n",Res1);*/
        //printf("temp %f \n",temp);
        
        if (banderaSUM > 0){    //BOTON AUMENTAR FRECUENCIA
            banderaSUM = 0;
            if (threshold_freq < 10){
                threshold_freq++;
            }
        }
        if (banderaRES > 0){    //BOTON DISMINUIR FRECUECIA
            banderaRES = 0;
            if (threshold_freq > 1){
                threshold_freq--;
            }
        }
        if (xQueueReceive(uart_queue,(void *)&event,pdMS_TO_TICKS(100))){   //RECEPCION UART
            bzero(data, BUF_SIZE);
            switch (event.type){    //LECTURA Y RECONOCIMIENTO DE COMANDOS
            case UART_DATA:
                uart_read_bytes(UART_NUM,data,event.size,pdMS_TO_TICKS(100));
                if (strstr(data,comando_get_temp))
                {
                    len = sprintf((char *)&message_to_send[0], " Valor Temperatura: %f\n", temp);
                    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
                }
                if (strstr(data,comando_get_rangos)){
			        len = sprintf((char *)&message_to_send[0], " Rangos de temperatura:\n");
                    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
                    len = sprintf((char *)&message_to_send[0], " Rojo: Max: %d, Min: %d \n",rang_max_R,rang_min_R);
                    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
                    len = sprintf((char *)&message_to_send[0], " Azul: Max: %d, Min: %d \n",rang_max_B,rang_min_B);
                    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
                    len = sprintf((char *)&message_to_send[0], " Verde: Max: %d, Min: %d \n",rang_max_G,rang_min_G);
                    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
		        }
                if (strstr(data,comando_get_thresh)){
                    len = sprintf((char *)&message_to_send[0], " Threshold: %d\n", threshold_temp);
                    uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
                }
                confirmTH = 0;
                confirmMR = 0;
                confirmmR = 0;
                confirmMG = 0;
                confirmmG = 0;
                confirmMB = 0;
                confirmmB = 0;
                for (int i = 0;i < 12; i++){
                    if ((confirmTH == 9)&&(i == 9)){
                        for (int j = i; j < 12;j++){
                            if (j - 9 == 0){
                                num = 10 * (data[j] - 48);
                            }
                            if (j - 9 == 1){
                                if (data[j] == '$'){
                                    num = data[j-1]-48;
                                }else{
                                    num = num + data[j]-48;
                                }
                                i = 13;
                            }
                        }
                        threshold_temp = num;
                    }
                    if ((confirmMR == 9)&&(i == 9)){
                        for (int j = i; j < 12;j++){
                            if (j - 9 == 0){
                                num = 10 * (data[j] - 48);
                            }
                            if (j - 9 == 1){
                                if (data[j] == '$'){
                                    num = data[j-1]-48;
                                }else{
                                    num = num + data[j]-48;
                                }
                                i = 13;
                            }
                        }
                        if (num > rang_min_R){
                            rang_max_R = num;
                        }else{
                            len = sprintf((char *)&message_to_send[0], " El maximo de temperatura debe ser mayor al minimo \n");
                            uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
                        }
                    }
                    if ((confirmmR == 9)&&(i == 9)){
                        for (int j = i; j < 12;j++){
                            if (j - 9 == 0){
                                num = 10*(data[j]-48);
                            }
                            if (j - 9 == 1){
                                if (data[j] == '$'){
                                    num = data[j-1]-48;
                                }else{
                                    num = num + data[j]-48;
                                }
                                i = 13;
                            }
                        }
                        if (num < rang_max_R){
                            rang_min_R = num;
                        }else{
                            len = sprintf((char *)&message_to_send[0], " El minimo de temperatura debe ser menor al maximo \n");
                            uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
                        }
                    }
                    if ((confirmMG == 9)&&(i == 9)){
                        for (int j = i; j < 12;j++){
                            if (j - 9 == 0){
                                num = 10*(data[j]-48);
                            }
                            if (j - 9 == 1){
                                if (data[j] == '$'){
                                    num = data[j-1]-48;
                                }else{
                                    num = num + data[j]-48;
                                }
                                i = 13;
                            }
                        }
                        if (num > rang_min_G){
                            rang_max_G = num;
                        }else{
                            len = sprintf((char *)&message_to_send[0], " El maximo de temperatura debe ser mayor al minimo \n");
                            uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
                        }
                    }
                    if ((confirmmG == 9)&&(i == 9)){
                        for (int j = i; j < 12;j++){
                            if (j - 9 == 0){
                                num = 10*(data[j]-48);
                            }
                            if (j - 9 == 1){
                                if (data[j] == '$'){
                                    num = data[j-1]-48;
                                }else{
                                    num = num + data[j]-48;
                                }
                                i = 13;
                            }
                        }
                        if (num < rang_max_G){
                            rang_min_G = num;
                        }else{
                            len = sprintf((char *)&message_to_send[0], " El minimo de temperatura debe ser menor al maximo \n");
                            uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
                        }
                    }
                    if ((confirmMB == 9)&&(i == 9)){
                        for (int j = i; j < 12;j++){
                            if (j - 9 == 0){
                                num = 10*(data[j]-48);
                            }
                            if (j - 9 == 1){
                                if (data[j] == '$'){
                                    num = data[j-1]-48;
                                }else{
                                    num = num + data[j]-48;
                                }
                                i = 13;
                            }
                        }
                        if (num > rang_min_B){
                            rang_max_B = num;
                        }else{
                            len = sprintf((char *)&message_to_send[0], " El maximo de temperatura debe ser mayor al minimo \n");
                            uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
                        }
                    }
                    if ((confirmmB == 9)&&(i == 9)){
                        for (int j = i; j < 11;j++){
                            if (j - 9 == 0){
                                num = 10*(data[j]-48);
                            }
                            if (j - 9 == 1){
                                if (data[j] == '$'){
                                    num = data[j-1]-48;
                                }else{
                                    num = num + data[j]-48;
                                }
                                i = 13;
                            }
                        }
                        if (num < rang_max_B){
                            rang_min_B = num;
                        }else{
                            len = sprintf((char *)&message_to_send[0], " El minimo de temperatura debe ser menor al maximo \n");
                            uart_write_bytes(UART_NUM, (const char*) message_to_send, len);
                        }
                    }
                    if (change_thresh[i] == data [i]){
				        confirmTH++;
			        }
                    if (comparaTMR[i] == data [i]){
				        confirmMR++;
			        }
                    if (comparaTmR[i] == data [i]){
                        confirmmR++;
                    }
                    if (comparaTMG[i] == data [i]){
				        confirmMG++;
			        }
                    if (comparaTmG[i] == data [i]){
                        confirmmG++;
                    }
                    if (comparaTMB[i] == data [i]){
				        confirmMB++;
			        }
                    if (comparaTmB[i] == data [i]){
                        confirmmB++;
                    }
                }
                uart_flush(UART_NUM);
                break;
            default:
                break;
            }
        }
        
        //ENCENDIDOS Y APAGADOS DEL RGB SEGUN LA TEMPERATURA
        if ((temp > rang_min_G)&&(temp < rang_max_G)){
            gpio_set_level(ledG, 1);
        }else{
            gpio_set_level(ledG, 0);
        }
        if ((temp > rang_min_B)&&(temp < rang_max_B)){
            gpio_set_level(ledB, 1);
        }else{
            gpio_set_level(ledB, 0);
        }
        if ((temp > rang_min_R)&&(temp < rang_max_R)){
            gpio_set_level(ledR, 1);
        }else{
            gpio_set_level(ledR, 0);
        }
    }
}

esp_err_t init_led(void)            //INICIALIZACION LEDS
{
    gpio_reset_pin(led0);
    gpio_set_direction(led0, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ledR);
    gpio_set_direction(ledR, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ledB);
    gpio_set_direction(ledB, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ledG);
    gpio_set_direction(ledG, GPIO_MODE_OUTPUT);
    return ESP_OK;
}

esp_err_t init_timer(void)          //INICIALIZACION TIMER
{
    ESP_LOGI(tag, "Timer iniciando timer.");
    xTimers = xTimerCreate("Timer",                   // Just a text name, not used by the kernel.
                           (pdMS_TO_TICKS(interval)), // The timer period in ticks.
                           pdTRUE,                    // The timers will auto-reload themselves when they expire.
                           (void *)timerId,           // Assign each timer a unique id equal to its array index.
                           vTimerCallback             // Each timer calls the same callback when it expires.
    );
    if (xTimers == NULL)
    {
        // The timer was not created.
        ESP_LOGI(tag, "Timer no creado.");
    }
    else
    {
        // Start the timer.  No block time is specified, and even if one was
        // it would be ignored because the scheduler has not yet been
        // started.
        if (xTimerStart(xTimers, 0) != pdPASS)
        {
            // The timer could not be set into the Active state.
            ESP_LOGI(tag, "The timer could not be set into the Active state");
        }
    }
    return ESP_OK;
}

esp_err_t init_ADC(void){           //INICIALIZACION ADC
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    return ESP_OK;
}

esp_err_t init_isr(void){           //INICIALIZACION INTERRUPCIONES
    gpio_config_t pISRConfig;
    pISRConfig.pin_bit_mask = ((1ULL << BotonSUM)|(1ULL << BotonRES));
    pISRConfig.mode = GPIO_MODE_DEF_INPUT;
    pISRConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pISRConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    pISRConfig.intr_type = GPIO_INTR_NEGEDGE;
    
    gpio_config(&pISRConfig); 
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BotonSUM,isr_handlerSUM,NULL);
    gpio_isr_handler_add(BotonRES,isr_handlerRES,NULL);
    return ESP_OK;
}

void isr_handlerSUM(void *args){    //FUNCION BANDERA BOTON SUMA
    banderaSUM++;
}

void isr_handlerRES(void *args){    //FUNCION BANDERA BOTON RESTA
    banderaRES++;
}
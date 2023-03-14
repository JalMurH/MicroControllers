/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DS18B20_Pin GPIO_PIN_3
#define DS18B20_GPIO_Port GPIOA
#define TTP223B_Pin GPIO_PIN_4
#define TTP223B_GPIO_Port GPIOA
#define SPI_Pin GPIO_PIN_5
#define SPI_GPIO_Port GPIOA
#define TTP223BA6_Pin GPIO_PIN_6
#define TTP223BA6_GPIO_Port GPIOA
#define TTP223BA7_Pin GPIO_PIN_7
#define TTP223BA7_GPIO_Port GPIOA
#define TTP223BB0_Pin GPIO_PIN_0
#define TTP223BB0_GPIO_Port GPIOB
#define MX25R6435FMIL0_Pin GPIO_PIN_1
#define MX25R6435FMIL0_GPIO_Port GPIOB
#define MX25R6435FMIL0B2_Pin GPIO_PIN_2
#define MX25R6435FMIL0B2_GPIO_Port GPIOB
#define CP2102_Pin GPIO_PIN_15
#define CP2102_GPIO_Port GPIOB
#define CP2102A8_Pin GPIO_PIN_8
#define CP2102A8_GPIO_Port GPIOA
#define IN_S66TATRGB_Pin GPIO_PIN_15
#define IN_S66TATRGB_GPIO_Port GPIOA
#define IN_S66TATRGBB3_Pin GPIO_PIN_3
#define IN_S66TATRGBB3_GPIO_Port GPIOB
#define IN_S66TATRGBB4_Pin GPIO_PIN_4
#define IN_S66TATRGBB4_GPIO_Port GPIOB
#define IN_S66TATRGBB5_Pin GPIO_PIN_5
#define IN_S66TATRGBB5_GPIO_Port GPIOB
#define IN_S66TATRGBB6_Pin GPIO_PIN_6
#define IN_S66TATRGBB6_GPIO_Port GPIOB
#define IN_S66TATRGBB7_Pin GPIO_PIN_7
#define IN_S66TATRGBB7_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

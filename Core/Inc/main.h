/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define BMP3_SENSOR_OK                                      UINT8_C(0)
#define BMP3_COMMUNICATION_ERROR_OR_WRONG_DEVICE            UINT8_C(10)
#define BMP3_TRIMMING_DATA_OUT_OF_BOUND                     UINT8_C(20)
#define BMP3_TEMPERATURE_BOUND_WIRE_FAILURE_OR_MEMS_DEFECT  UINT8_C(30)
#define BMP3_PRESSURE_BOUND_WIRE_FAILURE_OR_MEMS_DEFECT     UINT8_C(31)
#define BMP3_IMPLAUSIBLE_TEMPERATURE                        UINT8_C(40)
#define BMP3_IMPLAUSIBLE_PRESSURE                           UINT8_C(41)
/* 0 degree celsius */
#define BMP3_MIN_TEMPERATURE  INT16_C(0)

/* 40 degree celsius */
#define BMP3_MAX_TEMPERATURE  INT16_C(4000)

/* 900 hecto Pascals */
#define BMP3_MIN_PRESSURE     UINT32_C(90000)

/* 1100 hecto Pascals */
#define BMP3_MAX_PRESSURE     UINT32_C(110000)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NSS_Pin GPIO_PIN_4
#define NSS_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define NRFCE_Pin GPIO_PIN_4
#define NRFCE_GPIO_Port GPIOC
#define NRFIRQ_Pin GPIO_PIN_5
#define NRFIRQ_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define GP0_Pin GPIO_PIN_12
#define GP0_GPIO_Port GPIOB
#define GP1_Pin GPIO_PIN_13
#define GP1_GPIO_Port GPIOB
#define GP2_Pin GPIO_PIN_14
#define GP2_GPIO_Port GPIOB
#define GP3_Pin GPIO_PIN_15
#define GP3_GPIO_Port GPIOB
#define SPARE0_Pin GPIO_PIN_3
#define SPARE0_GPIO_Port GPIOB
#define SPARE1_Pin GPIO_PIN_4
#define SPARE1_GPIO_Port GPIOB
#define SPARE2_Pin GPIO_PIN_5
#define SPARE2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

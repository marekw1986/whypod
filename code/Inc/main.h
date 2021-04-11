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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

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
#define VOL_DN_Pin GPIO_PIN_2
#define VOL_DN_GPIO_Port GPIOE
#define SW_A_Pin GPIO_PIN_3
#define SW_A_GPIO_Port GPIOE
#define SW_A_EXTI_IRQn EXTI3_IRQn
#define KEY_1_Pin GPIO_PIN_2
#define KEY_1_GPIO_Port GPIOC
#define KEY_2_Pin GPIO_PIN_3
#define KEY_2_GPIO_Port GPIOC
#define KEY_3_Pin GPIO_PIN_0
#define KEY_3_GPIO_Port GPIOA
#define LOCK_SWITCH_Pin GPIO_PIN_1
#define LOCK_SWITCH_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define PWR_BTN_Pin GPIO_PIN_10
#define PWR_BTN_GPIO_Port GPIOB
#define T_IRQ_Pin GPIO_PIN_12
#define T_IRQ_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_13
#define LCD_RST_GPIO_Port GPIOD
#define LCD_BACKLIGHT_Pin GPIO_PIN_6
#define LCD_BACKLIGHT_GPIO_Port GPIOC
#define ENUSB_Pin GPIO_PIN_7
#define ENUSB_GPIO_Port GPIOC
#define USB_FAULT_Pin GPIO_PIN_8
#define USB_FAULT_GPIO_Port GPIOA
#define CHARGING_Pin GPIO_PIN_10
#define CHARGING_GPIO_Port GPIOA
#define SDIO_CD_Pin GPIO_PIN_3
#define SDIO_CD_GPIO_Port GPIOD
#define SW_B_Pin GPIO_PIN_8
#define SW_B_GPIO_Port GPIOB
#define ROT_PUSH_Pin GPIO_PIN_9
#define ROT_PUSH_GPIO_Port GPIOB
#define VOL_UP_Pin GPIO_PIN_0
#define VOL_UP_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

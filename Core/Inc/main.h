/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
typedef struct{
    uint16_t adc_values[8];
    uint16_t pot_positions[4];
    uint8_t pot_states[4];
    uint16_t d_btns;
    uint8_t f_btns;
    uint8_t encoder;
    uint32_t counter;
    int16_t count;
    uint8_t speed;
    uint32_t systicks;
} data_t;

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
#define RE_BTN_Pin GPIO_PIN_13
#define RE_BTN_GPIO_Port GPIOC
#define POT1_A_Pin GPIO_PIN_0
#define POT1_A_GPIO_Port GPIOA
#define POT1_B_Pin GPIO_PIN_1
#define POT1_B_GPIO_Port GPIOA
#define POT2_A_Pin GPIO_PIN_2
#define POT2_A_GPIO_Port GPIOA
#define POT2_B_Pin GPIO_PIN_3
#define POT2_B_GPIO_Port GPIOA
#define POT3_A_Pin GPIO_PIN_4
#define POT3_A_GPIO_Port GPIOA
#define POT3_B_Pin GPIO_PIN_5
#define POT3_B_GPIO_Port GPIOA
#define POT4_A_Pin GPIO_PIN_6
#define POT4_A_GPIO_Port GPIOA
#define POT4_B_Pin GPIO_PIN_7
#define POT4_B_GPIO_Port GPIOA
#define D_BTN_11_Pin GPIO_PIN_0
#define D_BTN_11_GPIO_Port GPIOB
#define D_BTN_3_Pin GPIO_PIN_1
#define D_BTN_3_GPIO_Port GPIOB
#define D_BTN_10_Pin GPIO_PIN_2
#define D_BTN_10_GPIO_Port GPIOB
#define D_BTN_9_Pin GPIO_PIN_10
#define D_BTN_9_GPIO_Port GPIOB
#define D_BTN_2_Pin GPIO_PIN_11
#define D_BTN_2_GPIO_Port GPIOB
#define D_BTN_1_Pin GPIO_PIN_12
#define D_BTN_1_GPIO_Port GPIOB
#define F_BTN_2_Pin GPIO_PIN_13
#define F_BTN_2_GPIO_Port GPIOB
#define D_BTN_4_Pin GPIO_PIN_14
#define D_BTN_4_GPIO_Port GPIOB
#define D_BTN_12_Pin GPIO_PIN_15
#define D_BTN_12_GPIO_Port GPIOB
#define RE_A_Pin GPIO_PIN_8
#define RE_A_GPIO_Port GPIOA
#define RE_B_Pin GPIO_PIN_9
#define RE_B_GPIO_Port GPIOA
#define D_BTN_13_Pin GPIO_PIN_10
#define D_BTN_13_GPIO_Port GPIOA
#define D_BTN_5_Pin GPIO_PIN_11
#define D_BTN_5_GPIO_Port GPIOA
#define D_BTN_14_Pin GPIO_PIN_12
#define D_BTN_14_GPIO_Port GPIOA
#define D_BTN_6_Pin GPIO_PIN_6
#define D_BTN_6_GPIO_Port GPIOF
#define F_BTN_3_Pin GPIO_PIN_7
#define F_BTN_3_GPIO_Port GPIOF
#define D_BTN_15_Pin GPIO_PIN_15
#define D_BTN_15_GPIO_Port GPIOA
#define D_BTN_7_Pin GPIO_PIN_3
#define D_BTN_7_GPIO_Port GPIOB
#define D_BTN_16_Pin GPIO_PIN_4
#define D_BTN_16_GPIO_Port GPIOB
#define D_BTN_8_Pin GPIO_PIN_5
#define D_BTN_8_GPIO_Port GPIOB
#define F_BTN_1_Pin GPIO_PIN_8
#define F_BTN_1_GPIO_Port GPIOB
#define F_BTN_4_Pin GPIO_PIN_9
#define F_BTN_4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

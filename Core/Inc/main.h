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
#define BTN_LONG_PRESS 4000 // 3s
#define ENCODER_WHOLE_ROTATION 30 // 30 steps per whole rotation
//total size of data_t is 48 bytes, transfer speed on i2c with 400kHz is 400kbps/8 = 50kB/s, so 48 bytes will be transferred in 0.96ms
typedef struct{
    uint16_t pot_adc_values[8]; // raw adc values
    uint16_t pot_positions[4]; // absolute position 0..1023
    uint8_t pot_states[4]; // BIT0: fwd, BIT1: bwd, BIT2: fast
    uint16_t d_btns; // BIT0-15: D1-D16
    uint16_t d_btns_long_press; // BIT0-15: D1-D16
    uint8_t f_btns; // BIT0: F1, BIT1: F2, BIT2: F3, BIT3: F4
    uint8_t f_btns_long_press; // BIT0: F1, BIT1: F2, BIT2: F3, BIT3: F4
    uint8_t encoder_state; // BIT0: button press, BIT1: long button press, BIT2: forward, BIT3: backward, BIT4: medium, BIT5: fast
    uint32_t encoder_counter; // 32 bit counter
    uint8_t encoder_absolute_pos; // 0 .. ENCODER_WHOLE_ROTATION-1
    uint8_t encoder_speed; // velocity of encoder
    uint32_t systicks; // timestamp
} ui_data_t;
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

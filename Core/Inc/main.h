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
    // 16-bit
    uint16_t d_btns; // BIT0-15: D1-D16
    uint16_t d_btns_long_press; // BIT0-15: D1-D16
    // function buttons are (0: F1, 1: F2, 2: POT1 (left), 3: POT2, 4: POT3, 5: POT4 (right))
    // 6-bit
    uint8_t f_btns;
    uint8_t f_btns_long_press;
    // mcl buttons are (0: MCL_LEFT, 1: MCL_DOWN, 2: MCL_RIGHT, 3: MCL_UP, 4: MCL_A, 5: MCL_B, 6: MCL_X, 7: MCL_Y, 8: MCL_P, 9: MCL_R, 10: MCL_S1, 11: MCL_S2)
    // 12-bit
    uint16_t mcl_btns;
    uint16_t mcl_btns_long_press;
    int16_t accelerometer[3];
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

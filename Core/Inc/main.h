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
    uint8_t f_btns; // BIT0: F1, BIT1: F2, BIT2: F3, BIT3: F4, BIT4: F5
    uint8_t f_btns_long_press; // BIT0: F1, BIT1: F2, BIT2: F3, BIT3: F4, BIT4: F5
    uint16_t mcl_btns; // BIT0: MCL1, BIT1: MCL2, BIT2: MCL3, BIT3: MCL4, BIT4: MCL5, BIT5: MCL6, BIT6: MCL7, BIT7: MCL8, BIT8: MCL9, BIT9: MCL10, BIT10: MCL11, BIT11: MCL12
    uint16_t mcl_btns_long_press; // BIT0: MCL1, BIT1: MCL2, BIT2: MCL3, BIT3: MCL4, BIT4: MCL5, BIT5: MCL6, BIT6: MCL7, BIT7: MCL8, BIT8: MCL9, BIT9: MCL10, BIT10: MCL11, BIT11: MCL12
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "endless_pot.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO uint32_t Transfer_Direction = 0;
__IO uint32_t i2c_transfer_complete = 0;
__IO uint32_t adc_dma_complete = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static ui_data_t data; // active data
static ui_data_t t_d_buffer; // transfer data buffer
static uint16_t adc_vals[8]; // adc values, dma buffer
static endless_pot_t pots[4]; // endless pots states
static uint32_t d_btn_timestamps[16]; // timestamps for buttons
static uint32_t f_btn_timestamps[5]; // timestamps for buttons
static uint32_t mcl_btn_timestamps[13]; // timestamps for buttons
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    memset(&data, 0, sizeof(ui_data_t));
    memset(&t_d_buffer, 0, sizeof(ui_data_t));

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

    // clock to 48MHz
    // Enable HSI (8 MHz internal clock)
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait for HSI to be ready

    // Configure Flash latency
    FLASH->ACR |= FLASH_ACR_LATENCY; // 1 wait state for 48 MHz

    // Configure PLL
    RCC->CFGR &= ~RCC_CFGR_PLLSRC; // Select HSI/2 as PLL source
    RCC->CFGR &= ~RCC_CFGR_PLLMUL; // Clear PLL multiplier bits
    RCC->CFGR |= RCC_CFGR_PLLMUL12; // Set PLL multiplier to 12

    RCC->CR |= RCC_CR_PLLON; // Enable PLL
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL to stabilize

    // Select PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL; // Select PLL as SYSCLK source
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait for switch

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    //HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
    //HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    if (HAL_I2C_EnableListen_IT(&hi2c2) != HAL_OK){
        /* Transfer error in reception process */
        Error_Handler();
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    // data fields
    uint32_t ticks_counter = 0;
    uint32_t encoder_old_count = 0;
    uint32_t encoder_old_count_2 = 0;
    uint8_t encoder_pre_state = 0;
    uint32_t encoder_button_timestamp = 0;
    for (int i=0;i<4;i++){
        endless_pot_init(&pots[i]);
    }
    HAL_Delay(10);
    // restart adc dma
    HAL_ADC_Start_DMA(&hadc, (uint32_t*) adc_vals, 8);
    while (1){
        // get ports A, B, C, and F
        uint16_t port_a_din = GPIOA->IDR;
        uint16_t port_b_din = GPIOB->IDR;
        uint16_t port_c_din = GPIOC->IDR;
        uint16_t port_d_din = GPIOD->IDR;
        uint16_t port_f_din = GPIOF->IDR;

        // data button 0-15 mapping is (PF0, PC14, PB9, PB8, PB6, PC12, PD2, PB3, PF1, PC15, PC13, PB7, PB5, PB4, PC11, PC10)
        data.d_btns = 0;
        data.d_btns |= (port_f_din & (1 << 0)) ? 0x01 : 0; // PF0
        data.d_btns |= (port_c_din & (1 << 14)) ? 0x02 : 0; // PC14
        data.d_btns |= (port_b_din & (1 << 9)) ? 0x04 : 0; // PB9
        data.d_btns |= (port_b_din & (1 << 8)) ? 0x08 : 0; // PB8
        data.d_btns |= (port_b_din & (1 << 6)) ? 0x10 : 0; // PB6
        data.d_btns |= (port_c_din & (1 << 12)) ? 0x20 : 0; // PC12
        data.d_btns |= (port_d_din & (1 << 2)) ? 0x40 : 0; // PD2
        data.d_btns |= (port_b_din & (1 << 3)) ? 0x80 : 0; // PB3
        data.d_btns |= (port_f_din & (1 << 1)) ? 0x100 : 0; // PF1
        data.d_btns |= (port_c_din & (1 << 15)) ? 0x200 : 0; // PC15
        data.d_btns |= (port_c_din & (1 << 13)) ? 0x400 : 0; // PC13
        data.d_btns |= (port_b_din & (1 << 7)) ? 0x800 : 0; // PB7
        data.d_btns |= (port_b_din & (1 << 5)) ? 0x1000 : 0; // PB5
        data.d_btns |= (port_b_din & (1 << 4)) ? 0x2000 : 0; // PB4
        data.d_btns |= (port_c_din & (1 << 11)) ? 0x4000 : 0; // PC11
        data.d_btns |= (port_c_din & (1 << 10)) ? 0x8000 : 0; // PC10
        // invert the buttons
        data.d_btns = ~data.d_btns;

        // check if data buttons are long pressed
        for (int i=0;i<16;i++){
            if (data.d_btns & (1 << i)){
                if (d_btn_timestamps[i] == 0){
                    d_btn_timestamps[i] = HAL_GetTick();
                }
                else if (HAL_GetTick() - d_btn_timestamps[i] > BTN_LONG_PRESS){
                    data.d_btns_long_press |= (1 << i);
                }
            }
            else{
                d_btn_timestamps[i] = 0;
                data.d_btns_long_press &= ~(1 << i);
            }
        }

        // function buttons are (PB0, PC5, PB2, PB1, PB12)
        data.f_btns = 0;
        data.f_btns |= (port_b_din & (1 << 0)) ? 0x01 : 0; // PB0
        data.f_btns |= (port_c_din & (1 << 5)) ? 0x02 : 0; // PC5
        data.f_btns |= (port_b_din & (1 << 2)) ? 0x04 : 0; // PB2
        data.f_btns |= (port_b_din & (1 << 1)) ? 0x08 : 0; // PB1
        data.f_btns |= (port_b_din & (1 << 12)) ? 0x10 : 0; // PB12
        data.f_btns = ~data.f_btns;
        data.f_btns = data.f_btns & 0x1F; // only 5 bits

        // check if function buttons are long pressed
        for (int i=0;i<5;i++){
            if (data.f_btns & (1 << i)){
                if (f_btn_timestamps[i] == 0){
                    f_btn_timestamps[i] = HAL_GetTick();
                }
                else if (HAL_GetTick() - f_btn_timestamps[i] > BTN_LONG_PRESS){
                    data.f_btns_long_press |= (1 << i);
                }
            }
            else{
                f_btn_timestamps[i] = 0;
                data.f_btns_long_press &= ~(1 << i);
            }
        }

        // mcl buttons are (PC0, PC1, PC2, PB14, PB13, PA12, PA11, PF6, PF7, PC3, PF4, PF5, PC4)
        data.mcl_btns = 0;
        data.mcl_btns |= (port_c_din & (1 << 0)) ? 0x01 : 0; // PC0
        data.mcl_btns |= (port_c_din & (1 << 1)) ? 0x02 : 0; // PC1
        data.mcl_btns |= (port_c_din & (1 << 2)) ? 0x04 : 0; // PC2
        data.mcl_btns |= (port_b_din & (1 << 14)) ? 0x08 : 0; // PB14
        data.mcl_btns |= (port_b_din & (1 << 13)) ? 0x10 : 0; // PB13
        data.mcl_btns |= (port_a_din & (1 << 12)) ? 0x20 : 0; // PA12
        data.mcl_btns |= (port_a_din & (1 << 11)) ? 0x40 : 0; // PA11
        data.mcl_btns |= (port_f_din & (1 << 6)) ? 0x80 : 0; // PF6
        data.mcl_btns |= (port_f_din & (1 << 7)) ? 0x100 : 0; // PF7
        data.mcl_btns |= (port_c_din & (1 << 3)) ? 0x200 : 0; // PC3
        data.mcl_btns |= (port_f_din & (1 << 4)) ? 0x400 : 0; // PF4
        data.mcl_btns |= (port_f_din & (1 << 5)) ? 0x800 : 0; // PF5
        data.mcl_btns |= (port_c_din & (1 << 4)) ? 0x1000 : 0; // PC4
        data.mcl_btns = ~data.mcl_btns;
        data.mcl_btns = data.mcl_btns & 0x1FFF; // only 13 bits

        for (int i=0;i<13;i++){
            if (data.mcl_btns & (1 << i)){
                if (mcl_btn_timestamps[i] == 0){
                    mcl_btn_timestamps[i] = HAL_GetTick();
                }
                else if (HAL_GetTick() - mcl_btn_timestamps[i] > BTN_LONG_PRESS){
                    data.mcl_btns_long_press |= (1 << i);
                }
            }
            else{
                mcl_btn_timestamps[i] = 0;
                data.mcl_btns_long_press &= ~(1 << i);
            }
        }

        /*
        // encoder positions
        data.encoder_counter = __HAL_TIM_GET_COUNTER(&htim1);
        data.encoder_counter >>= 1;
        data.encoder_absolute_pos = data.encoder_counter % ENCODER_WHOLE_ROTATION;
        // get velocity every 100ms
        uint32_t ticks = HAL_GetTick();
        if (ticks - ticks_counter >= 100){
            data.encoder_speed = data.encoder_counter > encoder_old_count
                             ? data.encoder_counter - encoder_old_count
                             : encoder_old_count - data.encoder_counter;
            encoder_old_count = data.encoder_counter;
            ticks_counter = ticks;
        }

        // encoder button is PC13, assign to btns bit 0
        data.encoder_state = (port_c_din & 0x2000) >> 13;
        data.encoder_state = ~data.encoder_state;
        data.encoder_state = data.encoder_state & 0x01;
        if (data.encoder_state == 1 && !(encoder_pre_state & 0x01)){
            encoder_button_timestamp = HAL_GetTick();
        }
        if (data.encoder_state == 1 && (HAL_GetTick() - encoder_button_timestamp) > BTN_LONG_PRESS){
            data.encoder_state |= 0x02;
        }
        // rotating forward or backward, slow, med or fast?
        if (data.encoder_counter > encoder_old_count_2){
            // forward
            data.encoder_state |= 0x04;
        }
        else if (data.encoder_counter < encoder_old_count_2){
            // backward
            data.encoder_state |= 0x08;
        }
        encoder_old_count_2 = data.encoder_counter;
        if (data.encoder_speed > 20){
            // fast spinning
            data.encoder_state |= 0x20;
        }
        else if (data.encoder_speed > 10){
            // medium spinning
            data.encoder_state |= 0x10;
        }
        encoder_pre_state = data.encoder_state;
        */

        // wait for adc
        while (adc_dma_complete != 1);
        memcpy(data.pot_adc_values, adc_vals, sizeof(adc_vals));
        for (int i=0;i<4;i++){
            endless_pot_update(&pots[i], data.pot_adc_values[i*2], data.pot_adc_values[i*2 + 1]);
            data.pot_positions[i] = pots[i].angle >> (16 - ENDLESS_POT_RESOLUTION);
            if (i != 0) data.pot_positions[i] = 1023 - data.pot_positions[i]; // all except first inverted
            data.pot_states[i] = pots[i].state;
            if (i == 0){
                // swap first and second bit of pot state as movement inverted
                data.pot_states[i] = (data.pot_states[i] & 0xFFFFFFFC) | ((data.pot_states[i] & 0x02) >> 1) | ((data.pot_states[i] & 0x01) << 1);
            }
        }

        // wait for last i2c transfer request from rp2040 to complete
        while (i2c_transfer_complete != 1);
        i2c_transfer_complete = 0;

        // get systicks as timestamp for package
        data.systicks = HAL_GetTick();

        // grab data for transfer
        memcpy(&t_d_buffer, &data, sizeof(ui_data_t));
        HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_vals, 8);

        // start listening for i2c transfer

        if (HAL_I2C_EnableListen_IT(&hi2c2) != HAL_OK){
            /* Transfer error in reception process */
            Error_Handler();
        }


        //
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    adc_dma_complete = 1;
}


/**
  * @brief  Tx Transfer completed callback.
  *   I2cHandle: I2C handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* I2cHandle){
    i2c_transfer_complete = 1;
}


/**
  * @brief  Rx Transfer completed callback.
  *   I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* I2cHandle){
    i2c_transfer_complete = 1;
    // nothing to rx here
}


/**
  * @brief  Slave Address Match callback.
  *   hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  *   TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
  *   AddrMatchCode: Address Match Code
  * @retval None
ui  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef* hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
    Transfer_Direction = TransferDirection;
    if (Transfer_Direction != 0){
        /*##- Start the transmission process #####################################*/
        /* While the I2C in reception process, user can transmit data through
           "aTxBuffer" buffer */
        if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c2, (uint8_t*)&t_d_buffer, sizeof(ui_data_t), I2C_FIRST_AND_LAST_FRAME) !=
            HAL_OK){
            /* Transfer error in transmission process */
            Error_Handler();
        }
    }
    /*
     // nothing to rx here
    else{
        if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t*)aRxBuffer, RXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) !=
            HAL_OK){
            Error_Handler();
        }
    }
    */
}

/**
  * @brief  Listen Complete callback.
  *   hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef* hi2c){
}

/**
  * @brief  I2C error callbacks.
  *   I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* I2cHandle){
    /** Error_Handler() function is called when error occurs.
      * 1- When Slave doesn't acknowledge its address, Master restarts communication.
      * 2- When Master doesn't acknowledge the last data transferred, Slave doesn't care in this example.
      */
    if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF){
        Error_Handler();
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1){
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

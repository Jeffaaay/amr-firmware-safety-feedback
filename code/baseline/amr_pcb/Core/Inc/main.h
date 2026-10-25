/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_INT1_Pin GPIO_PIN_1
#define IMU_INT1_GPIO_Port GPIOB
#define RS485_WR2_Pin GPIO_PIN_15
#define RS485_WR2_GPIO_Port GPIOF
#define RS485_WR1_Pin GPIO_PIN_0
#define RS485_WR1_GPIO_Port GPIOG
#define IMU_CS_Pin GPIO_PIN_11
#define IMU_CS_GPIO_Port GPIOE
#define IMU_INT2_Pin GPIO_PIN_3
#define IMU_INT2_GPIO_Port GPIOB
#define LIGHT_RED_Pin GPIO_PIN_0
#define LIGHT_RED_GPIO_Port GPIOF
#define LIGHT_GREEN_Pin GPIO_PIN_6
#define LIGHT_GREEN_GPIO_Port GPIOE
#define LIGHT_BLUE_Pin GPIO_PIN_5
#define LIGHT_BLUE_GPIO_Port GPIOE
#define ESTOP_INPUT_Pin GPIO_PIN_15
#define ESTOP_INPUT_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#ifndef ENABLE_LIFT_CAN
#define ENABLE_LIFT_CAN  1
#endif

/* ------------------------------------------------------------------------
 * Jetson command-link UART abstraction.
 * The protocol code never names a concrete peripheral; change ONLY this block
 * to move the Jetson link to a different UART/pins. (USART1 PA9/PA10 stays the
 * debug console.) Selected per PCB: USART3, TX=PC10, RX=PC11, AF7.
 * ------------------------------------------------------------------------ */
#define JETSON_UART_INSTANCE          USART3
#define JETSON_UART_IRQ               USART3_IRQn
#define JETSON_UART_IRQn              USART3_IRQn
#define JETSON_UART_IRQHandler        USART3_IRQHandler
#define JETSON_UART_CLK_ENABLE()      __HAL_RCC_USART3_CLK_ENABLE()
#define JETSON_UART_CLK_DISABLE()     __HAL_RCC_USART3_CLK_DISABLE()
#define JETSON_UART_GPIO_PORT         GPIOC
#define JETSON_UART_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define JETSON_UART_TX_PIN            GPIO_PIN_10   /* PC10 -> Jetson RX */
#define JETSON_UART_RX_PIN            GPIO_PIN_11   /* PC11 <- Jetson TX */
#define JETSON_UART_AF                GPIO_AF7_USART3
#define JETSON_UART_DESC              "USART3 PC10(TX)/PC11(RX) AF7"
#define JETSON_UART_HANDLE            huart3   /* CubeMX-generated UART handle */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

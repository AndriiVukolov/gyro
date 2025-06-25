/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define A0_Pin GPIO_PIN_0
#define A0_GPIO_Port GPIOF
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOF
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOF
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOF
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOF
#define A5_Pin GPIO_PIN_5
#define A5_GPIO_Port GPIOF
#define SPI5_SCK_Pin GPIO_PIN_7
#define SPI5_SCK_GPIO_Port GPIOF
#define SPI5_MISO_Pin GPIO_PIN_8
#define SPI5_MISO_GPIO_Port GPIOF
#define SPI5_MOSI_Pin GPIO_PIN_9
#define SPI5_MOSI_GPIO_Port GPIOF
#define ENABLE_Pin GPIO_PIN_10
#define ENABLE_GPIO_Port GPIOF
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define SDNWE_Pin GPIO_PIN_0
#define SDNWE_GPIO_Port GPIOC
#define NCS_MEMS_SPI_Pin GPIO_PIN_1
#define NCS_MEMS_SPI_GPIO_Port GPIOC
#define CSX_Pin GPIO_PIN_2
#define CSX_GPIO_Port GPIOC
#define CS_ACCEL_Pin GPIO_PIN_3
#define CS_ACCEL_GPIO_Port GPIOC
#define MEMS_INT1_Pin GPIO_PIN_1
#define MEMS_INT1_GPIO_Port GPIOA
#define MEMS_INT2_Pin GPIO_PIN_2
#define MEMS_INT2_GPIO_Port GPIOA
#define B5_Pin GPIO_PIN_3
#define B5_GPIO_Port GPIOA
#define VSYNC_Pin GPIO_PIN_4
#define VSYNC_GPIO_Port GPIOA
#define G2_Pin GPIO_PIN_6
#define G2_GPIO_Port GPIOA
#define ACP_RST_Pin GPIO_PIN_7
#define ACP_RST_GPIO_Port GPIOA
#define OTG_FS_PSO_Pin GPIO_PIN_4
#define OTG_FS_PSO_GPIO_Port GPIOC
#define OTG_FS_OC_Pin GPIO_PIN_5
#define OTG_FS_OC_GPIO_Port GPIOC
#define R3_Pin GPIO_PIN_0
#define R3_GPIO_Port GPIOB
#define R6_Pin GPIO_PIN_1
#define R6_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define SDNRAS_Pin GPIO_PIN_11
#define SDNRAS_GPIO_Port GPIOF
#define A6_Pin GPIO_PIN_12
#define A6_GPIO_Port GPIOF
#define A7_Pin GPIO_PIN_13
#define A7_GPIO_Port GPIOF
#define A8_Pin GPIO_PIN_14
#define A8_GPIO_Port GPIOF
#define A9_Pin GPIO_PIN_15
#define A9_GPIO_Port GPIOF
#define A10_Pin GPIO_PIN_0
#define A10_GPIO_Port GPIOG
#define A11_Pin GPIO_PIN_1
#define A11_GPIO_Port GPIOG
#define D4_Pin GPIO_PIN_7
#define D4_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOE
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_10
#define D7_GPIO_Port GPIOE
#define D8_Pin GPIO_PIN_11
#define D8_GPIO_Port GPIOE
#define D9_Pin GPIO_PIN_12
#define D9_GPIO_Port GPIOE
#define D10_Pin GPIO_PIN_13
#define D10_GPIO_Port GPIOE
#define D11_Pin GPIO_PIN_14
#define D11_GPIO_Port GPIOE
#define D12_Pin GPIO_PIN_15
#define D12_GPIO_Port GPIOE
#define G4_Pin GPIO_PIN_10
#define G4_GPIO_Port GPIOB
#define G5_Pin GPIO_PIN_11
#define G5_GPIO_Port GPIOB
#define OTG_HS_ID_Pin GPIO_PIN_12
#define OTG_HS_ID_GPIO_Port GPIOB
#define VBUS_HS_Pin GPIO_PIN_13
#define VBUS_HS_GPIO_Port GPIOB
#define OTG_HS_DM_Pin GPIO_PIN_14
#define OTG_HS_DM_GPIO_Port GPIOB
#define OTG_HS_DP_Pin GPIO_PIN_15
#define OTG_HS_DP_GPIO_Port GPIOB
#define D13_Pin GPIO_PIN_8
#define D13_GPIO_Port GPIOD
#define D14_Pin GPIO_PIN_9
#define D14_GPIO_Port GPIOD
#define D15_Pin GPIO_PIN_10
#define D15_GPIO_Port GPIOD
#define TE_Pin GPIO_PIN_11
#define TE_GPIO_Port GPIOD
#define RDX_Pin GPIO_PIN_12
#define RDX_GPIO_Port GPIOD
#define WRX_DCX_Pin GPIO_PIN_13
#define WRX_DCX_GPIO_Port GPIOD
#define D0_Pin GPIO_PIN_14
#define D0_GPIO_Port GPIOD
#define D1_Pin GPIO_PIN_15
#define D1_GPIO_Port GPIOD
#define BA0_Pin GPIO_PIN_4
#define BA0_GPIO_Port GPIOG
#define BA1_Pin GPIO_PIN_5
#define BA1_GPIO_Port GPIOG
#define R7_Pin GPIO_PIN_6
#define R7_GPIO_Port GPIOG
#define DOTCLK_Pin GPIO_PIN_7
#define DOTCLK_GPIO_Port GPIOG
#define SDCLK_Pin GPIO_PIN_8
#define SDCLK_GPIO_Port GPIOG
#define HSYNC_Pin GPIO_PIN_6
#define HSYNC_GPIO_Port GPIOC
#define G6_Pin GPIO_PIN_7
#define G6_GPIO_Port GPIOC
#define STLINK_RX_Pin GPIO_PIN_9
#define STLINK_RX_GPIO_Port GPIOA
#define STLINK_TX_Pin GPIO_PIN_10
#define STLINK_TX_GPIO_Port GPIOA
#define R4_Pin GPIO_PIN_11
#define R4_GPIO_Port GPIOA
#define R5_Pin GPIO_PIN_12
#define R5_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TP_INT1_Pin GPIO_PIN_15
#define TP_INT1_GPIO_Port GPIOA
#define R2_Pin GPIO_PIN_10
#define R2_GPIO_Port GPIOC
#define ACCEL_INT1_Pin GPIO_PIN_11
#define ACCEL_INT1_GPIO_Port GPIOC
#define ACCEL_INT2_Pin GPIO_PIN_12
#define ACCEL_INT2_GPIO_Port GPIOC
#define D2_Pin GPIO_PIN_0
#define D2_GPIO_Port GPIOD
#define D3_Pin GPIO_PIN_1
#define D3_GPIO_Port GPIOD
#define G7_Pin GPIO_PIN_3
#define G7_GPIO_Port GPIOD
#define B2_Pin GPIO_PIN_6
#define B2_GPIO_Port GPIOD
#define G3_Pin GPIO_PIN_10
#define G3_GPIO_Port GPIOG
#define B3_Pin GPIO_PIN_11
#define B3_GPIO_Port GPIOG
#define B4_Pin GPIO_PIN_12
#define B4_GPIO_Port GPIOG
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOG
#define LD4_Pin GPIO_PIN_14
#define LD4_GPIO_Port GPIOG
#define SDNCAS_Pin GPIO_PIN_15
#define SDNCAS_GPIO_Port GPIOG
#define USERPIN_Pin GPIO_PIN_4
#define USERPIN_GPIO_Port GPIOB
#define SDCKE1_Pin GPIO_PIN_5
#define SDCKE1_GPIO_Port GPIOB
#define SDNE1_Pin GPIO_PIN_6
#define SDNE1_GPIO_Port GPIOB
#define B6_Pin GPIO_PIN_8
#define B6_GPIO_Port GPIOB
#define B7_Pin GPIO_PIN_9
#define B7_GPIO_Port GPIOB
#define NBL0_Pin GPIO_PIN_0
#define NBL0_GPIO_Port GPIOE
#define NBL1_Pin GPIO_PIN_1
#define NBL1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define BUTTON_PIN  GPIO_PIN_0
#define BUTTON_PORT GPIOA
#define CUR_HIDE    "\e[?25l" // hide cursor
#define CUR_SHOW    "\e[?25h" // show cursor

#define SPI_TIMEOUT 100
/**
  * @brief  GYROSCOPE SPI Interface pins
  */
#define GYRO_CS_PIN                GPIO_PIN_1 /* PC.01 */
#define GYRO_CS_PORT               GPIOC      /* GPIOC */
#define GYRO_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()
#define GYRO_CS_GPIO_CLK_DISABLE() __HAL_RCC_GPIOC_CLK_DISABLE()

#define GYRO_INT_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define GYRO_INT_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define GYRO_INT_GPIO_PORT          GPIOA      /* GPIOA */
#define GYRO_INT1_PIN               GPIO_PIN_1 /* PA.01 */
#define GYRO_INT1_EXTI_IRQn         EXTI1_IRQn
#define GYRO_INT2_PIN               GPIO_PIN_2 /* PA.02 */
#define GYRO_INT2_EXTI_IRQn         EXTI2_IRQn

/**
  * @brief  ACCELEROMETER SPI Interface pins
  */
#define ACCEL_CS_PIN                 GPIO_PIN_3 /* PC.01 */
#define ACCEL_CS_PORT                GPIOC      /* GPIOC */
#define ACCEL_CS_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOC_CLK_ENABLE()
#define ACCEL_CS_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOC_CLK_DISABLE()
#define ACCEL_INT_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define ACCEL_INT_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define ACCEL_INT_GPIO_PORT          GPIOC       /* GPIOA */
#define ACCEL_INT1_PIN               GPIO_PIN_11 /* PA.01 */
#define ACCEL_INT1_EXTI_IRQn         EXTI1_IRQn
#define ACCEL_INT2_PIN               GPIO_PIN_12 /* PA.02 */
#define ACCEL_INT2_EXTI_IRQn         EXTI2_IRQn

/* Chip Select macro definition */
#define GYRO_SPI_EN() \
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_SPI_DIS() \
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET)

#define ACCEL_SPI_EN() \
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_RESET)
#define ACCEL_SPI_DIS() \
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_SET)
typedef struct {
    const char *str;        //string task sending to terminal
    uint8_t     led_num;    //led task blinking
    uint16_t    led_period; //led period (ms)
} task_param_t;

typedef enum { TASK_SEND_TEXT, TASK_SEND_NUMBER } queue_data_source_t;

typedef struct {
    uint32_t            nmb;
    queue_data_source_t src;
} queue_element_t;

void print(const char *str);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

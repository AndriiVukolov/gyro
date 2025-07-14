/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "crc.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "gyro_I3G4250D.h"
#include "accel.h"
#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>
#include "task.h"
#include "timers.h"
#include "queue.h"

#include "gui.h"
//#include "ili9341.h"
//#include "stm32f429i_discovery_lcd.h"
//#include "lcd.h"

//#define USE_HAL_DRIVERS
//#define HAL_RTC_MODULE_ENABLED

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define mainDELAY_LOOP_COUNT 0xFFFFFFFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//static gyroError_t   gerr         = gyroOk;
//static accel_error_t aerr         = ACCEL_OK;
//static uint8_t       timerIntFlag = 1;
//static float         g_val[3]     = { 0 };
//static uint32_t      indicator    = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void PERIF_SPI_MspInit(SPI_HandleTypeDef *hspi);
static void PERIF_SPI_Init(void);
static void PERIF_IO_Init(void);

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

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_FMC_Init();
    MX_USART1_UART_Init();
    MX_CRC_Init();
    MX_SPI5_Init();
    MX_DMA2D_Init();
    MX_I2C3_Init();
    MX_LTDC_Init();
    /* USER CODE BEGIN 2 */

    //rtc_init(&gyro_rtc);
    //PERIF_IO_Init();
    gui_init();
    gui_start();
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* Call init function for freertos objects (in cmsis_os2.c) */
    MX_FREERTOS_Init();

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        //        print(CUR_HIDE);
        //        if (indicator < 10000)
        //            indicator++;
        //        else
        //            indicator = 0;
        //        char strBuf[200] = { 0 };
        //        gyroReadVal(&g1, g_val);
        //        if (gerr == gyroOk)
        //            sprintf(strBuf,
        //                    " %.2f|%.2f|%.2f  *** %u|    \r",
        //                    g_val[0],
        //                    g_val[1],
        //                    g_val[2],
        //
        //                    indicator);
        //
        //        else
        //            sprintf(strBuf,
        //                    "  ACCEL read error!                                                 \r");
        //
        //        print(strBuf);

        //        memset(strBuf, 0, sizeof(strBuf));
        //        HAL_Delay(100);

        //        print(CUR_SHOW);
    }
    /* USER CODE END 3 */
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
    //  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    //  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    //
    //  /** Configure the main internal regulator output voltage
    //  */
    //  __HAL_RCC_PWR_CLK_ENABLE();
    //  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    //
    //  /** Initializes the RCC Oscillators according to the specified parameters
    //  * in the RCC_OscInitTypeDef structure.
    //  */
    //  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    //  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    //  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    //  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    //  RCC_OscInitStruct.PLL.PLLM = 4;
    //  RCC_OscInitStruct.PLL.PLLN = 72;
    //  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    //  RCC_OscInitStruct.PLL.PLLQ = 3;
    //  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    //  {
    //    Error_Handler();
    //  }
    //
    //  /** Initializes the CPU, AHB and APB buses clocks
    //  */
    //  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    //                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    //  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    //  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    //  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    //  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    //
    //  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    //  {
    //    Error_Handler();
    //  }
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
	     clocked below the maximum system frequency, to update the voltage scaling value
	     regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 360;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Activate the Over-Drive mode */
    HAL_PWREx_EnableOverDrive();

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	     clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/* USER CODE BEGIN 4 */

//void vApplicationIdleHook(void)
//{
//    counter++;
//}

static void PERIF_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable SPIx clock */
    __HAL_RCC_SPI5_CLK_ENABLE();

    /* Enable DISCOVERY_SPI GPIO clock */
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* configure SPI SCK, MOSI and MISO */
    GPIO_InitStructure.Pin       = (GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull      = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed     = GPIO_SPEED_MEDIUM;
    GPIO_InitStructure.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);
}

static void PERIF_SPI_Init(void)
{
    //  if (HAL_SPI_GetState(&hspi5) == HAL_SPI_STATE_RESET)
    //  {
    /* SPI configuration -----------------------------------------------------*/
    hspi5.Instance = SPI5;
    /* SPI baudrate is set to 5.6 MHz (PCLK2/SPI_BaudRatePrescaler = 90/16 = 5.625 MHz)

       - l3gd20 SPI interface max baudrate is 10MHz for write/read
       - PCLK2 frequency is set to 90 MHz
    */
    hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;

    /* On STM32F429I-Discovery, LCD ID cannot be read then keep a common configuration */
    /* for LCD and GYRO (SPI_DIRECTION_2LINES) */

    hspi5.Init.Direction      = SPI_DIRECTION_2LINES;
    hspi5.Init.CLKPhase       = SPI_PHASE_1EDGE;
    hspi5.Init.CLKPolarity    = SPI_POLARITY_LOW;
    hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi5.Init.CRCPolynomial  = 7;
    hspi5.Init.DataSize       = SPI_DATASIZE_8BIT;
    hspi5.Init.FirstBit       = SPI_FIRSTBIT_MSB;
    hspi5.Init.NSS            = SPI_NSS_SOFT;
    hspi5.Init.TIMode         = SPI_TIMODE_DISABLED;
    hspi5.Init.Mode           = SPI_MODE_MASTER;

    HAL_SPI_Init(&hspi5);
    PERIF_SPI_MspInit(&hspi5);
    //  }
}
static void PERIF_IO_Init(void)
{
    GPIO_InitTypeDef GYRO_GPIO_InitStructure;

    /* Configure the Gyroscope Control pins ------------------------------------*/
    /* Enable CS GPIO clock and Configure GPIO PIN for Gyroscope Chip select */
    GYRO_CS_GPIO_CLK_ENABLE();
    GYRO_GPIO_InitStructure.Pin   = GYRO_CS_PIN;
    GYRO_GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GYRO_GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GYRO_GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GYRO_CS_PORT, &GYRO_GPIO_InitStructure);

    GYRO_SPI_DIS();

    GPIO_InitTypeDef ACCEL_GPIO_InitStructure;

    /* Configure the Gyroscope Control pins ------------------------------------*/
    /* Enable CS GPIO clock and Configure GPIO PIN for Gyroscope Chip select */
    ACCEL_CS_GPIO_CLK_ENABLE();
    ACCEL_GPIO_InitStructure.Pin   = ACCEL_CS_PIN;
    ACCEL_GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    ACCEL_GPIO_InitStructure.Pull  = GPIO_NOPULL;
    ACCEL_GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(ACCEL_CS_PORT, &ACCEL_GPIO_InitStructure);

    /* Deselect: Chip Select high */

    ACCEL_SPI_DIS();

    /* Enable INT1, INT2 GPIO clock and Configure GPIO PINs to detect Interrupts */
    GYRO_INT_GPIO_CLK_ENABLE();
    GYRO_GPIO_InitStructure.Pin   = GYRO_INT1_PIN | GYRO_INT2_PIN;
    GYRO_GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
    GYRO_GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GYRO_GPIO_InitStructure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GYRO_INT_GPIO_PORT, &GYRO_GPIO_InitStructure);

    /* Enable INT1, INT2 GPIO clock and Configure GPIO PINs to detect Interrupts */
    ACCEL_INT_GPIO_CLK_ENABLE();
    ACCEL_GPIO_InitStructure.Pin   = ACCEL_INT1_PIN | ACCEL_INT2_PIN;
    ACCEL_GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
    ACCEL_GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    ACCEL_GPIO_InitStructure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(ACCEL_INT_GPIO_PORT, &ACCEL_GPIO_InitStructure);

    PERIF_SPI_Init();
}

void green_led_on(void)
{
    HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_SET);
}

void green_led_off(void)
{
    HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_RESET);
}

void green_led_toggle(void)
{
    HAL_GPIO_TogglePin(GPIOG, LD3_Pin);
}

void red_led_on(void)
{
    HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_SET);
}

void red_led_off(void)
{
    HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_RESET);
}

void red_led_toggle(void)
{
    HAL_GPIO_TogglePin(GPIOG, LD4_Pin);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
    /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

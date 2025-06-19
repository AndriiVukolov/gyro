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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gyro_I3G4250D.h"
#include "accel.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>
#include "task.h"

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
static gyro_t  g1 = { 0 };
static accel_t a1 = { 0 };

static uint8_t       receiveBuf[32]  = { 0 };
static uint8_t       transmitBuf[32] = { 0 };
static uint8_t       intFlag         = 0;
static float         gyro_bias[3]    = { 0 };
static float         accel_bias[3]   = { 0 };
static gyroError_t   gerr            = gyroOk;
static accel_error_t aerr            = ACCEL_OK;
static unsigned int  indicator       = 0;
static uint8_t       slaveAdd        = 0b11010000;
static uint8_t       timerIntFlag    = 1;
static uint16_t      timCounter      = 0;
static uint8_t       dummyByte       = 0;
static uint8_t       accel_drdy      = 0;

static interrupt_t  aint1, aint2;
static tap_dir_t    a1_tap_single_dir, a1_tap_double_dir;
static tap_thr_t    a1_tap_single_thr, a1_tap_double_thr;
static fifo_state_t fst;
static uint8_t      data_qty;
static uint8_t      int_src[6];
static uint8_t      accel_id;
static uint8_t      accel_drdy_flag = 0;

static const char *task_text_1 = "Task 1 ";
static const char *task_text_2 = "Task 2 ";
static const char *task_text_3 = "Periodic Task 3 \r\n";

uint32_t counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

static void print(const char *str);
static gyroError_t
func_read_gyro_spi(uint8_t startAdd, uint16_t len, uint8_t *store);
static gyroError_t
func_write_gyro_spi(uint8_t startAdd, uint16_t len, uint8_t *store);
static accel_error_t
func_read_accel_spi(uint8_t startAdd, uint8_t len, uint8_t *store);
static accel_error_t
func_write_accel_spi(uint8_t startAdd, uint8_t len, uint8_t *store);

static void PERIF_SPI_MspInit(SPI_HandleTypeDef *hspi);
static void PERIF_SPI_Init(void);
static void PERIF_IO_Init(void);

static void func_test_task(void *pvParameters);
static void func_test_periodic(void *pvParameters);

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
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    MX_CRC_Init();
    MX_SPI5_Init();
    /* USER CODE BEGIN 2 */
    //=============================================================GYRO Init
    PERIF_IO_Init();

    g1.funcReadRegs  = func_read_gyro_spi;
    g1.funcWriteRegs = func_write_gyro_spi;
    gerr             = gyroInit(&g1);
    if (gerr != gyroOk)
        print("Gyro init error! \r\n");
    //==========================================================END OF GYRO INIT

    //=============================================================ACCEL INIT

    a1.data_write = func_write_accel_spi;
    a1.data_read  = func_read_accel_spi;
    aint1.ibyte   = 0;
    //aint1.ibit.drdy         = 1;
    aint2.ibyte             = 0;
    a1_tap_single_dir.cbyte = 7; // all directions 0b00000111
    a1_tap_double_dir.cbyte = 7; // all directions

    a1_tap_single_thr.x_thr = 4; // 4/32 = 1/8g
    a1_tap_single_thr.y_thr = 4;
    a1_tap_single_thr.z_thr = 4;

    a1_tap_double_thr.x_thr = 4; // 4/32 = 1/8g
    a1_tap_double_thr.y_thr = 4;
    a1_tap_double_thr.z_thr = 4;

    aerr = accel_init(&a1); // set defines

    accel_filter_set(&a1, 0, 0, fs_4); //Filter mode set
    accel_int1_set(&a1, aint1);        //Interrupt 1 set
    accel_int2_set(&a1, aint2);        //Interrupt 2 set
    //    accel_free_fall_set(&a1, FF_250, 1); //Free fall detection set
    //    accel_wake_up_set(&a1, 16, 2);       //Wake up mode set
    accel_int_mode_set(&a1, PULSED); //Interrupt mode set
                                     //    accel_single_tap_set(&a1,
    //                         &a1_tap_single_dir,
    //                         &a1_tap_single_thr,
    //                         0,
    //                         0,
    //                         4);                   //Single tap set
    //    accel_orientation_param_set(&a1, STHR_60); //Orientation detection set
    accel_int_disable(&a1);
    accel_interface_set(&a1, SPI_4);
    accel_bdu_set(&a1, 1);
    //=========================================================FIFO settings
    accel_autoincrement_set(&a1, ACCEL_DISABLE);
    //accel_fifo_set(&a1, FIFO_OFF, 0);
    accel_fifo_set(&a1, FIFO_CONTIN, 32);
    //accel_slp_mode_set(&a1, 0, 0);
    //=========================================================Turn on accel
    accel_power_mode_set(&a1,
                         LOW_POWER_MODE,
                         LP_MODE_2,
                         IPM_50,
                         LN_OFF); //Power mode set

    accel_id = accel_id_get(&a1);

    if (aerr != ACCEL_OK)
        print("Accel init error! \r\n");
    //==========================================================END OF ACCEL INIT

    /* USER CODE END 2 */

    /* Call init function for freertos objects (in cmsis_os2.c) */
    MX_FREERTOS_Init();

    xTaskCreate(func_test_task,
                "TASK1",
                configMINIMAL_STACK_SIZE,
                (void *)task_text_1,
                1,
                NULL);
    xTaskCreate(func_test_task,
                "TASK2",
                configMINIMAL_STACK_SIZE,
                (void *)task_text_2,
                1,
                NULL);
    xTaskCreate(func_test_periodic,
                "TASK3",
                configMINIMAL_STACK_SIZE,
                (void *)task_text_3,
                2,
                NULL);

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

        //        if (gerr == gyroOk)
        //            sprintf((char *)strBuf,
        //                    " %.2f|%.2f|%.2f  *** %u| ID - %u   \r",
        //                    accel_bias[0],
        //                    accel_bias[1],
        //                    accel_bias[2],
        //
        //                    indicator,
        //                    accel_id);
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
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
  */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 4;
    RCC_OscInitStruct.PLL.PLLN       = 72;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void func_test_task(void *pvParameters)
{
    char *ptr_str;
    char  str_buf[50] = { 0 };

    ptr_str = (char *)pvParameters;

    while (1) {
        HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
        sprintf(str_buf, "%s / %lu \r\n", ptr_str, counter);
        print(str_buf);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void func_test_periodic(void *pvParameters)
{
    char        *ptr_str;
    portTickType start_tick;

    ptr_str    = (char *)pvParameters;
    start_tick = xTaskGetTickCount();

    while (1) {
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        print(ptr_str);
        //vTaskDelay(500 / portTICK_RATE_MS);
        vTaskDelayUntil(&start_tick, pdMS_TO_TICKS(700));
    }
}

void vApplicationIdleHook(void)
{
    if (counter < 0xFFFFFFFF)
        counter++;
    else
        counter = 0;
}

static gyroError_t
func_read_gyro_spi(uint8_t startAdd, uint16_t len, uint8_t *store)
{
    HAL_StatusTypeDef err    = HAL_OK;
    gyroError_t       gerr   = gyroOk;
    uint8_t           rxByte = 0;
    uint8_t           txByte = 0;

    if (len > 1) {
        txByte = (READ << 7) | (MULTIPLY << 6) | (startAdd & 0x3F);
    } else {
        txByte = (READ << 7) | (SINGLE << 6) | (startAdd & 0x3F);
    }

    GYRO_SPI_EN();
    err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);

    for (uint8_t x = 0; x < len; x++) {
        txByte = 0;
        err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
        store[x] = rxByte;
        rxByte   = 0;
    }
    GYRO_SPI_DIS();
    if (err != HAL_OK)
        gerr = gyroCommError;

    return gerr;
}

static gyroError_t
func_write_gyro_spi(uint8_t startAdd, uint16_t len, uint8_t *store)
{
    HAL_StatusTypeDef err    = HAL_OK;
    gyroError_t       gerr   = gyroOk;
    uint8_t           rxByte = 0;
    uint8_t           txByte = 0;

    GYRO_SPI_EN();
    if (len > 1) {
        txByte = (WRITE << 7) | (MULTIPLY << 6) | (startAdd & 0x3F);
    } else {
        txByte = (WRITE << 7) | (SINGLE << 6) | (startAdd & 0x3F);
    }
    err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
    for (uint8_t x = 0; x < len; x++) {
        txByte = store[x];
        rxByte = 0;
        err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
    }
    GYRO_SPI_DIS();
    if (err != HAL_OK)
        gerr = gyroCommError;
    return gerr;
}

void print(const char *str)
{
    if (HAL_UART_Transmit(&huart1, (const uint8_t *)str, strlen(str), 10) !=
        HAL_OK) {
        Error_Handler();
    }
}

static accel_error_t
func_read_accel_spi(uint8_t startAdd, uint8_t len, uint8_t *store)
{
    HAL_StatusTypeDef err    = HAL_OK;
    gyroError_t       gerr   = gyroOk;
    uint8_t           rxByte = 0;
    uint8_t           txByte = 0;

    if (len > 1) {
        txByte = (READ << 7) | (MULTIPLY << 6) | (startAdd & 0x3F);
    } else {
        txByte = (READ << 7) | (SINGLE << 6) | (startAdd & 0x3F);
    }

    ACCEL_SPI_EN();
    err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);

    for (uint8_t x = 0; x < len; x++) {
        txByte = 0;
        err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
        store[x] = rxByte;
        rxByte   = 0;
    }
    ACCEL_SPI_DIS();
    if (err != HAL_OK)
        gerr = gyroCommError;

    return gerr;
}

static accel_error_t
func_write_accel_spi(uint8_t startAdd, uint8_t len, uint8_t *store)
{
    HAL_StatusTypeDef err    = HAL_OK;
    gyroError_t       gerr   = gyroOk;
    uint8_t           rxByte = 0;
    uint8_t           txByte = 0;

    ACCEL_SPI_EN();
    if (len > 1) {
        txByte = (WRITE << 7) | (MULTIPLY << 6) | (startAdd & 0x3F);
    } else {
        txByte = (WRITE << 7) | (SINGLE << 6) | (startAdd & 0x3F);
    }
    err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
    for (uint8_t x = 0; x < len; x++) {
        txByte = store[x];
        rxByte = 0;
        err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
    }
    ACCEL_SPI_DIS();
    if (err != HAL_OK)
        gerr = gyroCommError;
    return gerr;
}

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
    hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;

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

    PERIF_SPI_MspInit(&hspi5);
    HAL_SPI_Init(&hspi5);
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
    ACCEL_GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
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
    if (htim->Instance == TIM1) {
        timerIntFlag = 1;
    }
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

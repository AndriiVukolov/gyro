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
#include "crc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gyro_I3G4250D.h"
#include "stm32f4xx_hal_def.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
gyro_t g1 = {0};

uint8_t receiveBuf[32] = {0};
uint8_t transmitBuf[32] = {0};
uint8_t intFlag = 0;
float bias[3] = {0};
gyroError_t gerr = gyroOk;
unsigned int indicator = 0;
uint8_t slaveAdd = 0b11010000;
uint8_t timerIntFlag = 1;
uint16_t timCounter = 0;
uint8_t dummyByte = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void print (const char * str);
static gyroError_t funcReadSPI(uint8_t startAdd, uint16_t len, uint8_t *store);
static gyroError_t funcWriteSPI(uint8_t startAdd, uint16_t len, uint8_t *store);
static void GYRO_SPI_MspInit(SPI_HandleTypeDef *hspi);
static void GYRO_SPI_Init(void);
static void GYRO_IO_Init(void);


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

  GYRO_IO_Init();

  g1.funcReadRegs = funcReadSPI;
  g1.funcWriteRegs = funcWriteSPI;

  //==========================================================GYRO SETTINGS

  //=====================REG1==============
  g1.DataRate = ODR100;
  g1.BandWidth = 3;
  g1.PDownModeEn = 1;
  g1.Zen = 1;
  g1.Yen = 1;
  g1.Xen = 1;
  //=====================REG2==============
  g1.HPFilterMode = 0; //Normal mode (reset by reading the REFERENCE/DATACAPTURE (25h) register)
  g1.HPCutOffCode = 0;
  //=====================REG3==============
  g1.Int1En = DISABLE;
  g1.Int1Boot = DISABLE;
  g1.IntActConfig = 0;
  g1.Int2En = DISABLE;
  g1.FIFOOvrnEn = DISABLE;
  g1.FIFOEmptyEn = DISABLE;
  //=====================REG4==============
  g1.Ble = 0;
  g1.FullScale = FS500;
  g1.SelfTestEn = DISABLE;
  g1.SPIModeSel = 0;
  //=====================REG5==============
  g1.Boot = 0;
  g1.FifoEn = 0;
  g1.HPFilterEn = 0;
  g1.Int1SelConf = 0;
  g1.OutSelConf = 0;
  //=====================INT1_CFG=========
  g1.Andor  = 0;
  g1.Lir  = 0;
  g1.ZHIE  = 0;
  g1.ZLIE = 0;
  g1.YHIE = 0;
  g1.YLIE = 0;
  g1.XHIE = 0;
  g1.XLIE = 0;
  //=====================INT1_DURATION====
  g1.Int1Wait = 0;
  g1.Int1Duration = 1;
  //=====================INT1_THRESHOLDS==
  g1.Int1XHighTr = 0;
  g1.Int1XLowTr = 0;
  g1.Int1YHighTr = 0;
  g1.Int1YLowTr = 0;
  g1.Int1ZHighTr = 0;
  g1.Int1ZLowTr = 0;
  //===================FIFO MODE SELECTION
  g1.FifoModeSel = 0;
  //=====================HighPath Filter Mode
  g1.HPFilterMode = HPNormal;
  g1.HPCutOffCode = HPCF0;


  //==========================================================END OF GYRO SETTINGS


  gerr = gyroInit(&g1);
  if (gerr != gyroOk) print("Gyro init error! \r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (indicator < 10000) indicator++;
	  else indicator = 0;
	  char strBuf[200] = {0};

//	  gerr = gyroReadID(&g1);
//	  gerr = gyroReadIntStatus(&g1);
//	  gerr = gyroReadReference(&g1);
//	  gerr = g1.funcReadRegs(ADD_REFERENCE, 1, &dummyByte);
//	  gerr = gyroReadAll(&g1, receiveBuf);
//	  gerr = g1.funcReadRegs(ADD_CTRL_REG1, 25, receiveBuf);
	  gerr = gyroReadVal(&g1, bias);

//      gerr = gyroReadStatus(&g1);
//      if (gerr == gyroOk) sprintf(strBuf, "  %u|%u|%u|%u|%u|%u|%u|%u|  \r", g1.zyxor, g1.zor, g1.yor, g1.xor, g1.zyxda, g1.zda, g1.yda, g1.xda);
//      else sprintf(strBuf, "  GYRO Error!                 \r");

//      gerr = g1.funcReadRegs(ADD_OUT_X_L, 1, &receiveBuf[0]);
//      gerr = g1.funcReadRegs(ADD_OUT_Y_L, 1, &receiveBuf[2]);
//      gerr = g1.funcReadRegs(ADD_OUT_Y_H, 1, &receiveBuf[3]);
//      gerr = g1.funcReadRegs(ADD_OUT_Z_L, 1, &receiveBuf[4]);
//      gerr = g1.funcReadRegs(ADD_OUT_Z_H, 1, &receiveBuf[5]);
//      gerr = g1.funcReadRegs(ADD_OUT_X_L, 1, &receiveBuf[0]);
//      gerr = g1.funcReadRegs(ADD_OUT_X_H, 1, &receiveBuf[1]);

      if (gerr == gyroOk) sprintf((char*)strBuf, " %-.2f|%-.2f|%-.2f       \r", bias[0], bias[1], bias[2]);
//      if (gerr == gyroOk) sprintf(strBuf, " <%u>  %u|%u|%u  %u         \r", g1.Id, g1.OutX, g1.OutY, g1.OutZ, indicator++);

//      if (gerr == gyroOk)
//      {
//    	  char oneByte[4] = {0};
//
//    	  sprintf(strBuf, "<%u> ", g1.Id);
//    	  for (uint8_t y = 0; y < 30; y++)
//    	  {
//    		  sprintf(oneByte, "%2X|", receiveBuf[y]);
//    		  strcat(strBuf, oneByte);
//    	  }
//    	  sprintf(oneByte, "%u\r", indicator);
//		  strcat(strBuf, oneByte);
//      }
      else sprintf(strBuf, "  GYRO read error!                                                 \r");
//      memset(receiveBuf, 0, sizeof(receiveBuf));
//      if (g1.zyxor)
//      {
//          sprintf(strBuf, "  GYRO overrun!  x:%u, y:%u, z:%u \r", g1.xor, g1.yor, g1.zor);
//      }

      //print(" \r");

      print(strBuf);
      memset(strBuf, 0, sizeof(strBuf));
      HAL_Delay(500);
//      if (timerIntFlag == 1)
//      {
//    	  timerIntFlag = 0;
//    	  timCounter++;
    	  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//    	  if (timCounter == 1000)
//    	  {
//    		  timCounter = 0;
//
//    	  }
//      }
//
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if (GPIO_Pin == MEMS_INT1_Pin)
//    {
//        intFlag = 1;
//    }
//
//    if (GPIO_Pin == MEMS_INT2_Pin)
//    {
//        intFlag = 1;
//    }
//
//}
static gyroError_t funcReadSPI(uint8_t startAdd, uint16_t len, uint8_t *store)
{
    HAL_StatusTypeDef err = HAL_OK;
    gyroError_t gerr = gyroOk;
    uint8_t rxByte = 0;
    uint8_t txByte = 0;

    if (len > 1)
    {
    	txByte = (READ << 7) | (MULTIPLY << 6) | (startAdd & 0x3F);
    }
    else
    {
    	txByte = (READ << 7) | (SINGLE << 6) | (startAdd & 0x3F);
    }

    SPI_EN();
    SPI_EN();
    err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);

    for(uint8_t x = 0; x < len; x++)
    {
    	txByte = 0;
		err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
		store[x] = rxByte;
		rxByte = 0;
    }
    SPI_DIS();
    if (err != HAL_OK) gerr = gyroCommError;

    return gerr;
}

static gyroError_t funcWriteSPI(uint8_t startAdd, uint16_t len, uint8_t *store)
{
    HAL_StatusTypeDef err = HAL_OK;
    gyroError_t gerr = gyroOk;
    uint8_t rxByte = 0;
    uint8_t txByte = 0;

    SPI_EN();
    SPI_EN();
    if (len > 1)
    {
        txByte = (WRITE << 7) | (MULTIPLY << 6) | (startAdd & 0x3F);
    }
    else
    {
    	txByte = (WRITE << 7) | (SINGLE << 6) | (startAdd & 0x3F);
    }
    err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
    for(uint8_t x = 0; x < len; x++)
    {
    	txByte = store[x];
    	rxByte = 0;
    	err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
    }
    SPI_DIS();
    if (err != HAL_OK) gerr = gyroCommError;
    return gerr;
}
void print (const char * str)
{
    if (HAL_UART_Transmit(&huart1, (const uint8_t*)str, strlen(str), 10) != HAL_OK)
    {
        Error_Handler();
    }
}

static void GYRO_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable SPIx clock */
  __HAL_RCC_SPI5_CLK_ENABLE();

  /* Enable DISCOVERY_SPI GPIO clock */
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* configure SPI SCK, MOSI and MISO */
  GPIO_InitStructure.Pin    = (GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
  GPIO_InitStructure.Mode   = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull   = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed  = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Alternate = GPIO_AF5_SPI5;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);
}

static void GYRO_SPI_Init(void)
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

    GYRO_SPI_MspInit(&hspi5);
    HAL_SPI_Init(&hspi5);
//  }
}
static void GYRO_IO_Init(void)
{
  GPIO_InitTypeDef GYRO_GPIO_InitStructure;

  /* Configure the Gyroscope Control pins ------------------------------------*/
  /* Enable CS GPIO clock and Configure GPIO PIN for Gyroscope Chip select */
  GYRO_CS_GPIO_CLK_ENABLE();
  GYRO_GPIO_InitStructure.Pin = GYRO_CS_PIN;
  GYRO_GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GYRO_GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GYRO_GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &GYRO_GPIO_InitStructure);

  /* Deselect: Chip Select high */
  GYRO_CS_HIGH();

  /* Enable INT1, INT2 GPIO clock and Configure GPIO PINs to detect Interrupts */
  GYRO_INT_GPIO_CLK_ENABLE();
  GYRO_GPIO_InitStructure.Pin = GYRO_INT1_PIN | GYRO_INT2_PIN;
  GYRO_GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GYRO_GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GYRO_GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GYRO_INT_GPIO_PORT, &GYRO_GPIO_InitStructure);

  GYRO_SPI_Init();
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
	if (htim->Instance == TIM1)
	{
		timerIntFlag = 1;
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

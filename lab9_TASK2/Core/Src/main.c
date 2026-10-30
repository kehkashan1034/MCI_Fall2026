/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float ax, ay, az;
    float offset_x, offset_y, offset_z;
    float roll, pitch;
} LSM_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LSM_READ_ADDR 0x33 // 8-bit read address
#define WHO_AM_I_REG  0x0F // WHO_AM_I register
#define EXPECTED_ID   0x33 // Expected response

#define LSM303_ADDR   (0x19U << 1)
#define CTRL_REG1_A   0x20U
#define CTRL_REG4_A   0x23U
#define OUT_X_L_A     0x28U
#define ACC_G_PER_LSB 0.004f
#define RAD_TO_DEG    57.2957795f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static LSM_Data sensor = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void myPrintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void myPrintf(const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/* ============================================================================
   TASK 2 FUNCTIONS (Commented out from active execution, but fully preserved)
   ============================================================================ */
/*
static HAL_StatusTypeDef Init_LSM(void)
{
    uint8_t config = 0x67;  // 200 Hz, all axes enabled, normal mode
    if (HAL_I2C_Mem_Write(&hi2c1, LSM303_ADDR, CTRL_REG1_A,
                          I2C_MEMADD_SIZE_8BIT, &config, 1, 100) != HAL_OK)
        return HAL_ERROR;

    config = 0x00;         // +/-2g, normal resolution
    if (HAL_I2C_Mem_Write(&hi2c1, LSM303_ADDR, CTRL_REG4_A,
                          I2C_MEMADD_SIZE_8BIT, &config, 1, 100) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(50);
    return HAL_OK;
}

static HAL_StatusTypeDef Read_LSM(LSM_Data *s)
{
    uint8_t data[6];
    if (HAL_I2C_Mem_Read(&hi2c1, LSM303_ADDR, OUT_X_L_A | 0x80U,
                        I2C_MEMADD_SIZE_8BIT, data, 6, 100) != HAL_OK)
        return HAL_ERROR;

    int16_t raw_x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    s->ax = (raw_x / 64) * ACC_G_PER_LSB - s->offset_x;
    s->ay = (raw_y / 64) * ACC_G_PER_LSB - s->offset_y;
    s->az = (raw_z / 64) * ACC_G_PER_LSB - s->offset_z;

    s->roll = atan2f(s->ay, sqrtf(s->ax*s->ax + s->az*s->az)) * RAD_TO_DEG;
    s->pitch = atan2f(-s->ax, sqrtf(s->ay*s->ay + s->az*s->az)) * RAD_TO_DEG;
    return HAL_OK;
}

static HAL_StatusTypeDef Offset_LSM(LSM_Data *s)
{
    float sx = 0, sy = 0, sz = 0;
    const int samples = 20;
    s->offset_x = s->offset_y = s->offset_z = 0.0f;

    for (int i = 0; i < samples; ++i)
    {
        if (Read_LSM(s) != HAL_OK)
            return HAL_ERROR;
        sx += s->ax;
        sy += s->ay;
        sz += s->az;
        HAL_Delay(20);
    }
    s->offset_x = sx / samples;
    s->offset_y = sy / samples;
    float avg_z = sz / samples;
    s->offset_z = avg_z - ((avg_z >= 0.0f) ? 1.0f : -1.0f);
    return HAL_OK;
}

static void Print_LSM(const LSM_Data *s)
{
    char msg[128];
    int len = snprintf(msg, sizeof(msg), "X: %.3f g | Y: %.3f g | Z: %.3f g | Roll: %.2f° | Pitch: %.2f°\r\n",
                      s->ax, s->ay, s->az, s->roll, s->pitch);
    if (len > 0)
    {
        if (len >= (int)sizeof(msg))
            len = sizeof(msg) - 1;
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, 100);
    }
}
*/
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(100); // sensor boot time[cite: 2]

  // ==========================================
  // TASK 1: Check WHO_AM_I register (ACTIVE)
  // ==========================================
  uint8_t who_am_i = 0;
  HAL_StatusTypeDef ret;

  ret = HAL_I2C_Mem_Read(&hi2c1, LSM_READ_ADDR, WHO_AM_I_REG,
                         I2C_MEMADD_SIZE_8BIT, &who_am_i, 1,
                         HAL_MAX_DELAY);

  if (ret != HAL_OK)
  {
      myPrintf("I2C ERROR: %d\r\n", ret);
  }
  else
  {
      myPrintf("WHO_AM_I = 0x%02X\r\n", who_am_i);
      if (who_am_i == EXPECTED_ID)
      {
          myPrintf("LSM303AGR detected successfully!\r\n");
      }
  }

  /*
  // ==========================================
  // TASK 2: Initialization & Calibration (COMMENTED OUT)
  // ==========================================
  if (Init_LSM() != HAL_OK)
  {
      myPrintf("LSM INIT FAILED\r\n");
      Error_Handler();
  }

  if (Offset_LSM(&sensor) != HAL_OK)
  {
      myPrintf("LSM CALIBRATION FAILED\r\n");
      Error_Handler();
  }
  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*
    // ==========================================
    // TASK 2: Continuous Read Loop (COMMENTED OUT)
    // ==========================================
    if (Read_LSM(&sensor) == HAL_OK)
    {
        Print_LSM(&sensor);
    }
    else
    {
        myPrintf("LSM READ FAILED\r\n");
    }
    */

    HAL_Delay(100);
  /* USER CODE END 3 */
  }
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
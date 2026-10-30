/* USER CODE BEGIN Header */
/**
  ********************************----------------------------------------------
  * @file           : main.c
  * @brief          : Main program body for Lab 9 Task 2 (Accelerometer Only)
  ********************************----------------------------------------------
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int16_t ax_raw, ay_raw, az_raw;
    float   ax_g, ay_g, az_g;
    float   acc_offset_x, acc_offset_y, acc_offset_z;
    float   pitch, roll;
} SensorData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- I2C Accelerometer (LSM303AGR) --- */
#define LSM_I2C_ADDR     0x32       // Base 7-bit address shifted left for HAL
#define CTRL_REG1_A      0x20
#define CTRL_REG4_A      0x23
#define OUT_X_L_A        0x28
#define OUT_X_H_A        0x29
#define OUT_Y_L_A        0x2A
#define OUT_Y_H_A        0x2B
#define OUT_Z_L_A        0x2C
#define OUT_Z_H_A        0x2D
#define ACCEL_SCALE      3.9f       // mg per LSB at +/-2g range
#define RAD_TO_DEG       57.2958f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SensorData sensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static int16_t Accel_ReadAxis(uint8_t reg_l, uint8_t reg_h);
void Accel_Init(void);
void Accel_Calibrate(void);
void Accel_Read(void);
void Sensor_Print(void);
void Sensor_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int16_t Accel_ReadAxis(uint8_t reg_l, uint8_t reg_h) {
    uint8_t lo, hi;
    HAL_I2C_Mem_Read(&hi2c1, LSM_I2C_ADDR, reg_l, I2C_MEMADD_SIZE_8BIT, &lo, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, LSM_I2C_ADDR, reg_h, I2C_MEMADD_SIZE_8BIT, &hi, 1, HAL_MAX_DELAY);
    return (int16_t)((hi << 8) | lo);
}

void Accel_Init(void) {
    uint8_t val = 0x67; // ODR 100Hz, normal mode, X/Y/Z enabled
    HAL_I2C_Mem_Write(&hi2c1, LSM_I2C_ADDR, CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
    val = 0x00; // normal mode, +-2g
    HAL_I2C_Mem_Write(&hi2c1, LSM_I2C_ADDR, CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}

void Accel_Calibrate(void) {
    float sx = 0, sy = 0, sz = 0;
    const int samples = 20;
    
    char msg[] = "Accel calibrating... keep still\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    for (int i = 0; i < samples; i++) {
        sx += (Accel_ReadAxis(OUT_X_L_A, OUT_X_H_A) >> 6);
        sy += (Accel_ReadAxis(OUT_Y_L_A, OUT_Y_H_A) >> 6);
        sz += (Accel_ReadAxis(OUT_Z_L_A, OUT_Z_H_A) >> 6);
        HAL_Delay(5);
    }
    sensor.acc_offset_x = sx / samples;
    sensor.acc_offset_y = sy / samples;
    sensor.acc_offset_z = (sz / samples) - (1000.0f / ACCEL_SCALE); // remove 1g gravity reference
}

void Accel_Read(void) {
    // Read registers and right-shift by 6 to isolate the 10-bit value
    sensor.ax_raw = Accel_ReadAxis(OUT_X_L_A, OUT_X_H_A) >> 6;
    sensor.ay_raw = Accel_ReadAxis(OUT_Y_L_A, OUT_Y_H_A) >> 6;
    sensor.az_raw = Accel_ReadAxis(OUT_Z_L_A, OUT_Z_H_A) >> 6;

    // Convert raw counts into human-readable gravity units (g)
    sensor.ax_g = (sensor.ax_raw * ACCEL_SCALE - sensor.acc_offset_x * ACCEL_SCALE) / 1000.0f;
    sensor.ay_g = (sensor.ay_raw * ACCEL_SCALE - sensor.acc_offset_y * ACCEL_SCALE) / 1000.0f;
    sensor.az_g = (sensor.az_raw * ACCEL_SCALE - (sensor.acc_offset_z * ACCEL_SCALE + 1000.0f)) / 1000.0f;

    // Calculate pitch angle
    sensor.pitch = atan2f(sensor.ax_g, sensor.az_g) * RAD_TO_DEG;
}

void Sensor_Print(void) {
    // Scale integers by 100 to print cleanly without floating-point linker constraints
    int ax_int = (int)(sensor.ax_g * 100);
    int ay_int = (int)(sensor.ay_g * 100);
    int az_int = (int)(sensor.az_g * 100);
    int pitch_int = (int)(sensor.pitch * 100);

    char buffer[128];
    int len = sprintf(buffer, "aX:%d | aY:%d | aZ:%d | Pitch:%d\r\n",
                      ax_int, ay_int, az_int, pitch_int);

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

void Sensor_Update(void) {
    Accel_Read();
    Sensor_Print();
}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);

  // Initialize and calibrate Accelerometer OUTSIDE the while loop
  Accel_Init();
  Accel_Calibrate();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Sensor_Update();
    HAL_Delay(100); // 100 ms delay interval for reliable UART pacing
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t *line)
{
}
#endif /* USE_FULL_ASSERT */
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
#include <math.h>          // Crucial para la función pow()
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "bmp280.h"        // Tu librería del sensor de presión
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
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
// --- Variables del BMP280 ---
BMP280_HandleTypedef bmp280;
const float PRESION_NIVEL_MAR = 101325.0f;
float temperatura, presion_Pa, humedad, altitud_m;

// --- Variables del MPU6050 ---
// Variables crudas (enteros de 16 bits)
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

// Variables finales convertidas (decimales)
float Ax, Ay, Az;
float Gx, Gy, Gz;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void Error_Handler(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

    // 1. Iniciar Pantalla
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(5, 25);
    ssd1306_WriteString("Iniciando...", Font_7x10, White);
    ssd1306_UpdateScreen();

    // 2. Iniciar BMP280
    bmp280.i2c = &hi2c1;
    bmp280.addr = BMP280_I2C_ADDRESS_0;
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_init(&bmp280, &params);

    // 3. Iniciar MPU6050
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(5, 25);
    ssd1306_WriteString("Iniciando MPU...", Font_7x10, White);
    ssd1306_UpdateScreen();
    HAL_Delay(1000);

    // 2. Despertar al MPU6050 (Por defecto inicia "dormido")
    // Escribimos un 0x00 en el registro de Power Management (0x6B)
    uint8_t Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &Data, 1, 1000);

    // Nota: Al despertar sin configurar nada más, el MPU6050 queda con
    // sensibilidades por defecto: +/- 2G (Acelerómetro) y +/- 250°/s (Giroscopio).

    HAL_Delay(1000); // Pausa para que todo se estabilice

    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
	    while (1)
	    {
	        char buffer_oled[64];

	        // --- 1. LEER BMP280 ---
	        if (bmp280_read_float(&bmp280, &temperatura, &presion_Pa, &humedad)) {
	            altitud_m = 44330.0f * (1.0f - pow((presion_Pa / PRESION_NIVEL_MAR), 0.1902949f));
	        }

	        // --- 2. LEER MPU6050 ---
	        // --- 2. LEER MPU6050 ---
	              uint8_t Data[14];

	              // Leer 14 bytes desde el registro 0x3B (ACCEL_XOUT_H)
	              HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x3B, 1, Data, 14, 1000);

	              // Ensamblar los bytes altos y bajos en enteros de 16 bits
	              Accel_X_RAW = (int16_t)(Data[0] << 8 | Data[1]);
	              Accel_Y_RAW = (int16_t)(Data[2] << 8 | Data[3]);
	              Accel_Z_RAW = (int16_t)(Data[4] << 8 | Data[5]);

	              Gyro_X_RAW = (int16_t)(Data[8] << 8 | Data[9]);
	              Gyro_Y_RAW = (int16_t)(Data[10] << 8 | Data[11]);
	              Gyro_Z_RAW = (int16_t)(Data[12] << 8 | Data[13]);

	              // Convertir a valores físicos (con decimales)
	              // Divisor 16384.0 para el acelerómetro (rango +/- 2g)
	              Ax = Accel_X_RAW / 16384.0f;
	              Ay = Accel_Y_RAW / 16384.0f;
	              Az = Accel_Z_RAW / 16384.0f;

	              // Divisor 131.0 para el giroscopio (rango +/- 250 grados/s)
	              Gx = Gyro_X_RAW / 131.0f;
	              Gy = Gyro_Y_RAW / 131.0f;
	              Gz = Gyro_Z_RAW / 131.0f;


	        // --- 3. MOSTRAR EN OLED ---
	        ssd1306_Fill(Black);

	        // Fila 1: Altura y Temperatura (BMP280)
	        sprintf(buffer_oled, "Alt:%.1fm T:%.0fC", altitud_m, temperatura);
	        ssd1306_SetCursor(2, 2);
	        ssd1306_WriteString(buffer_oled, Font_7x10, White);

	        // Separador visual
	        ssd1306_Line(0, 15, 128, 15, White);

	        // Fila 2: Acelerómetro X e Y
	        sprintf(buffer_oled, "Ax:%.2f Ay:%.2f", Ax, Ay); // Reemplaza Ax y Ay por tus variables
	        ssd1306_SetCursor(2, 20);
	        ssd1306_WriteString(buffer_oled, Font_7x10, White);

	        // Fila 3: Acelerómetro Z
	        sprintf(buffer_oled, "Az:%.2f Gs", Az); // Reemplaza Az por tu variable
	        ssd1306_SetCursor(2, 35);
	        ssd1306_WriteString(buffer_oled, Font_7x10, White);

	        // Fila 4: Giroscopio (resumen)
	        sprintf(buffer_oled, "Gx:%.0f Gy:%.0f", Gx, Gy); // Reemplaza Gx y Gy por tus variables
	        ssd1306_SetCursor(2, 50);
	        ssd1306_WriteString(buffer_oled, Font_7x10, White);

	        ssd1306_UpdateScreen();

	        // Un delay corto para que la pantalla se sienta fluida
	        HAL_Delay(100);
	    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

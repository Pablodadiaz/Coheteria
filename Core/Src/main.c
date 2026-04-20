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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include <stdio.h>
#include <string.h>
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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
typedef enum {
    ESTADO_ESPERA,
    ESTADO_ASCENSO,
    ESTADO_DESCENSO,
    ESTADO_ATERRIZADO
} EstadoVuelo;

EstadoVuelo estado_actual = ESTADO_ESPERA;
float altitud_base = 0.0f;
float altitud_actual = 0.0f;
float altitud_relativa = 0.0f;
float altitud_anterior = 0.0f;
float altitud_maxima = 0.0f;
float velocidad_actual = 0.0f;
float velocidad_ascenso_max = 0.0f;
float velocidad_descenso_max = 0.0f;
uint32_t tiempo_anterior = 0;

// Variables del Filtro Alpha-Beta
float alpha = 0.2f;  // Confiamos 20% en el sensor, 80% en la física
float beta = 0.01f;  // Corrección muy suave para la velocidad
float h_est = 0.0f;  // Altitud estimada (filtrada)
float v_est = 0.0f;  // Velocidad estimada (filtrada)

// Variables de seguridad
uint8_t contador_apogeo = 0; // Para exigir 3 confirmaciones

// Variables de redundancia por tiempo
uint32_t tiempo_despegue = 0;
// Tiempos máximos estimados de vuelo (¡Ajustar según simulador!)
const float TIEMPO_BACKUP_APOGEO = 15.0f;    // Dispara drogue a los 15 seg de vuelo sí o sí
const float TIEMPO_BACKUP_PRINCIPAL = 60.0f; // Dispara principal a los 60 seg de vuelo sí o sí

// Variables MPU6050 y Filtro de Media Móvil
#define NUM_MUESTRAS_ACCEL 5
float buffer_accel[NUM_MUESTRAS_ACCEL] = {0};
uint8_t indice_accel = 0;
float accel_z_filtrada = 0.0f;
float umbral_burnout = 0.5f;
uint8_t motor_encendido = 0;

uint8_t rechazos_consecutivos = 0; // Salvavidas del filtro
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
  BMP280_Init(&hi2c1);
  HAL_Delay(100);

  // Promediar altitud base antes de despegar
  for(int i = 0; i < 10; i++) {
      altitud_base += BMP280_ReadAltitude();
      HAL_Delay(50);
  }
  altitud_base /= 10.0f;
  tiempo_anterior = HAL_GetTick();

  // Inicializar MPU6050 (Configuración básica para +/- 8g)
  uint8_t check;
  uint8_t Data;
  HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x75, 1, &check, 1, 1000); // WHO_AM_I
  if (check == 0x68) {
      Data = 0;
      HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &Data, 1, 1000); // Salir de Sleep Mode
      Data = 0x10; // Configurar a +/- 8g
      HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x1C, 1, &Data, 1, 1000);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

          // === 1. LECTURAS DE SENSORES ===
          // Barómetro (BMP280)
          float altitud_cruda = BMP280_ReadAltitude() - altitud_base;

          // Acelerómetro (MPU6050 - Eje Z)
          uint8_t Rec_Data[2];
          int16_t Accel_Z_RAW;
          HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x3F, 1, Rec_Data, 2, 1000);
          Accel_Z_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
          float accel_z_cruda = Accel_Z_RAW / 4096.0f; // Convertir a Gs

          uint32_t tiempo_actual = HAL_GetTick();
          float delta_t = (tiempo_actual - tiempo_anterior) / 1000.0f;

          if (delta_t >= 0.05f) { // Ejecutar a 20 Hz

              // ACTUALIZAR TIEMPO SIEMPRE ACÁ (Fuera del filtro)
              tiempo_anterior = tiempo_actual;

              // === 2. FILTRADO DE DATOS ===

              // Filtro Media Móvil (Acelerómetro)
              buffer_accel[indice_accel] = accel_z_cruda;
              indice_accel++;
              if (indice_accel >= NUM_MUESTRAS_ACCEL) {
                  indice_accel = 0;
              }
              float suma_accel = 0.0f;
              for (int i = 0; i < NUM_MUESTRAS_ACCEL; i++) {
                  suma_accel += buffer_accel[i];
              }
              accel_z_filtrada = suma_accel / NUM_MUESTRAS_ACCEL;

              // Rechazo de Imposibles Físicos (Barómetro)
              float v_bruta = (altitud_cruda - h_est) / delta_t;

              // FILTRO CON ANTI-LOCKOUT INCLUIDO
              if ((v_bruta > -343.0f && v_bruta < 343.0f) || rechazos_consecutivos > 10) {

                  // Resincronización si veníamos rechazando datos
                  if (rechazos_consecutivos > 10) {
                      h_est = altitud_cruda;
                      v_est = 0.0f;
                  }
                  rechazos_consecutivos = 0;

                  // Filtro Alpha-Beta (Barómetro)
                  float h_pred = h_est + (v_est * delta_t);
                  float v_pred = v_est;
                  float error = altitud_cruda - h_pred;

                  h_est = h_pred + (alpha * error);
                  v_est = v_pred + ((beta / delta_t) * error);

                  // === ACTUALIZACIÓN DE TELEMETRÍA MÁXIMA ===
                  if (estado_actual != ESTADO_ESPERA) {
                      if (h_est > altitud_maxima) altitud_maxima = h_est;
                  }
                  if (estado_actual == ESTADO_ASCENSO) {
                      if (v_est > velocidad_ascenso_max) velocidad_ascenso_max = v_est;
                  }
                  if (estado_actual == ESTADO_DESCENSO) {
                      if (v_est < velocidad_descenso_max) velocidad_descenso_max = v_est;
                  }

                  // === 3. MÁQUINA DE ESTADOS (CON TRIPLE REDUNDANCIA) ===
                  switch (estado_actual) {
                      case ESTADO_ESPERA:
                          if (accel_z_filtrada > 2.0f || (h_est > 5.0f && v_est > 15.0f)) {
                              estado_actual = ESTADO_ASCENSO;
                              motor_encendido = 1;
                              tiempo_despegue = HAL_GetTick();
                          }
                          break;

                      case ESTADO_ASCENSO:
                      {
                          float tiempo_vuelo = (HAL_GetTick() - tiempo_despegue) / 1000.0f;

                          if (motor_encendido && accel_z_filtrada < umbral_burnout) {
                              motor_encendido = 0;
                          }

                          if ((altitud_maxima - h_est > 3.0f) && (v_est < -2.0f)) {
                              contador_apogeo++;
                              if (contador_apogeo >= 3) {
                                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // Drogue
                                  estado_actual = ESTADO_DESCENSO;
                              }
                          } else {
                              contador_apogeo = 0;
                          }

                          if (tiempo_vuelo >= TIEMPO_BACKUP_APOGEO) {
                              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // Drogue por Timer
                              estado_actual = ESTADO_DESCENSO;
                          }
                          break;
                      }

                      case ESTADO_DESCENSO:
                      {
                          float tiempo_vuelo = (HAL_GetTick() - tiempo_despegue) / 1000.0f;

                          if (h_est <= 250.0f && altitud_maxima > 250.0f) {
                              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // Principal
                              estado_actual = ESTADO_ATERRIZADO;
                          }

                          if (tiempo_vuelo >= TIEMPO_BACKUP_PRINCIPAL) {
                              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // Principal por Timer
                              estado_actual = ESTADO_ATERRIZADO;
                          }
                          break;
                      }

                      case ESTADO_ATERRIZADO:
                          // Apagamos los pirotécnicos para no quemar la placa
                          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
                          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

                          char reporte[120];
                          sprintf(reporte, "REPORTE DE VUELO\r\nAlt Max: %.2f m\r\nVel Max: %.2f m/s\r\n", altitud_maxima, velocidad_ascenso_max);
                          HAL_UART_Transmit(&huart1, (uint8_t*)reporte, strlen(reporte), 1000);
                          HAL_Delay(2000);
                          break;
                  }
              } else {
                  // Si el dato es físicamente imposible, contamos el rechazo
                  rechazos_consecutivos++;
              }
          }
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

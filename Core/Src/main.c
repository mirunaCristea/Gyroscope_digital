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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <math.h>


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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */


uint32_t adc_raw[3]={0,0,0}; // Buffer pentru citirea valorilor ADC (3 canale)
float adc_volt[3]; // Buffer pentru conversia valorilor ADC in volti (3 canale)
float adc_acc[3]; // Buffer pentru conversia valorilor ADC in acceleratie (3 canale)
float unghi[3]={0,0,0}; // Buffer pentru unghiurile de inclinare (3 canale)
float offset[3]={0,0,0}; // Buffer pentru offseturile ADC (3 canale)

float delta_t[3]={0,0,0}; // Buffer pentru timpul de citire a valorilor ADC (3 canale)


float gyro[3]={0,0,0}; // Buffer pentru valorile gyroscopului (3 canale)
  
float unghi_anterior[3]={0,0,0}; // Buffer pentru unghiurile de inclinare (3 canale)


uint32_t time_prev[3]={0,0,0}; // Timpul anterior
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


////SCRIS DE MINE











// Redefinirea funcției printf pentru a funcționa cu HAL_UART_Transmit
// Aceasta permite folosirea directă a printf în loc de HAL_UART_Transmit()
// Utile pentru debug și transmiterea datelor senzorilor

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch) 
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif



PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int _write(int fd, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  return len;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  // Initializeaza offseturile cu media valorilor ADC
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); // Inițializează HAL (Hardware Abstraction Layer)

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); // Configurează ceasul sistemului (PLL, frecvențe periferice)

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();  // Inițializează pinii GPIO
  MX_ADC1_Init();   // Inițializează convertorul analog-digital (ADC)
  MX_USART1_UART_Init(); // Inițializează UART-ul pentru comunicare serială
  MX_USB_PCD_Init(); // Inițializează USB 
  /* USER CODE BEGIN 2 */
  // Inițializezi offseturile cu 0



//////SCRIS DE MINE

//////SCRIS DE MINE
for (int i = 0; i < 3; i++) offset[i] = 0;
for (int n = 0; n < 100; n++) {
    for (int i = 0; i < 3; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);
        offset[i] += HAL_ADC_GetValue(&hadc1);
    }
}
for (int i = 0; i < 3; i++) 
{offset[i] /= 100.0f;
offset[i] = (offset[i] * 3.3f) / 4095.0f; // Conversie la voltaj
printf("Offset %d: %.3f V\r\n", i, offset[i]);
}
offset[2]=(offset[0]+offset[1])/2.0f; // Offset pentru Z
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  for (int i = 0; i < 3; ++i)
    {
        // 1. Citire ADC
        HAL_ADC_Start(&hadc1); // Incepe conversia ADC
        HAL_ADC_PollForConversion(&hadc1, 1); // Asteapta conversia
        adc_raw[i] = HAL_ADC_GetValue(&hadc1); // Citeste valoarea ADC

        // 2. Conversie la voltaj
        adc_volt[i] = (adc_raw[i] * 3.3f) / 4095.0f; // Conversie la voltaj (cu referință 3.3V fiindcă ADC-ul e alimentat la 3.3V)


        // 3. Conversie la acceleratie (g)
        
        adc_acc[i] = (adc_volt[i] - offset[i]) / 0.33f;  // Conversie la accelerație (g) – 0.33V/g  fiind rezolutia senzorului


        // 4. Calcul unghi (grade)
        unghi_anterior[i] = unghi[i];
        // Pentru X și Y folosește formula clasică de tilt (atenție la axa de referință!)
        if (i == 0)
            unghi[i] = atan(adc_acc[0]/adc_acc[2]) * 180.0f / 3.1415f; // Roll (X)
        else if (i == 1)
            unghi[i] = atan(adc_acc[1]/adc_acc[2]) * 180.0f / 3.1415f; // Pitch (Y)
        
        // 5. Calcul timp (ms)
        uint32_t time_now = HAL_GetTick(); // Obține timpul curent în milisecunde
        delta_t[i] = time_now - time_prev[i]; // Calculează diferența de timp
        time_prev[i] = time_now; // Salvează timpul curent pentru următoarea iterație
        
        HAL_Delay(100); // Delay pentru a evita citirea prea rapida a ADC-ului
        // 6. Calcul viteza unghiulara (grade/secunda)
        if (delta_t[i] > 0)
            gyro[i] = (unghi[i] - unghi_anterior[i]) / (delta_t[i] / 1000.0f); // Calcul viteza unghiulara (dps) 

        else
            gyro[i] = 0;
    }

    // Afisare CLARA
    HAL_Delay(1000);
    printf("RAW   -> X: %lu  Y: %lu  Z: %lu\r\n", adc_raw[0], adc_raw[1], adc_raw[2]);
   
    printf("VOLT  -> X: %.3f V  Y: %.3f V  Z: %.3f V\r\n", adc_volt[0], adc_volt[1], adc_volt[2]);
    
    printf("ACCEL -> X: %.3f g  Y: %.3f g  Z: %.3f g\r\n", adc_acc[0], adc_acc[1], adc_acc[2]);
    HAL_Delay(100);
    printf("ANGLE -> X: %.2f°   Y: %.2f°   Z: %.2f°\r\n", unghi[0], unghi[1], unghi[2]);
    
    printf("GYRO  -> X: %.2f dps Y: %.4f dps Z: %.4f dps\r\n", gyro[0], gyro[1], gyro[2]);
    printf("--------------------------------------------------\r\n");

    HAL_Delay(1000); // Poți ajusta delay-ul după nevoie



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

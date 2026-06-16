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
#include "UserConfig.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEVICE_ID                 1U
#define SAMPLE_INTERVAL_MS        5000U
#define ADC_REF_MV                3300U
#define ADC_MAX_VALUE             4095U
#define TMP117_ADDR               (0x48U << 1)
#define TMP117_TEMP_REG           0x00U
#define MLX90393_ADDR             (0x0CU << 1)
#define MLX90393_CMD_START_ZYX    0x3EU
#define MLX90393_CMD_READ_ZYX     0x4EU

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static void Enter_Stop(uint32_t sleep_time_s);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  读取 PA1/ADC_IN1 电压值，并换算成毫伏。
  * @note   当前按 ADC 参考电压 3.3V、12bit 分辨率直接换算；如果前端有分压电阻，需在此处乘以分压比例。
  * @retval 电压值，单位 mV。
  */
static uint16_t Read_Voltage_mV(void)
{
  uint32_t adc_value = 0U;

  if (HAL_ADC_Start(&hadc) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(&hadc, 20) == HAL_OK)
    {
      adc_value = HAL_ADC_GetValue(&hadc);
    }
    (void)HAL_ADC_Stop(&hadc);
  }

  return (uint16_t)((adc_value * ADC_REF_MV) / ADC_MAX_VALUE);
}

/**
  * @brief  通过 I2C1 读取 TMP117 温度。
  * @note   TMP117 地址为 0x48，温度寄存器为 0x00；返回值放大 100 倍，便于无浮点格式化。
  * @retval 温度 x100，例如 2534 表示 25.34 摄氏度。
  */
static int16_t Read_TMP117_Temp_x100(void)
{
  uint8_t reg = TMP117_TEMP_REG;
  uint8_t data[2] = {0};
  int16_t raw;

  if (HAL_I2C_Master_Transmit(&hi2c1, TMP117_ADDR, &reg, 1, 100) != HAL_OK)
  {
    return 0;
  }
  if (HAL_I2C_Master_Receive(&hi2c1, TMP117_ADDR, data, 2, 100) != HAL_OK)
  {
    return 0;
  }

  raw = (int16_t)((uint16_t)data[0] << 8 | data[1]);
  return (int16_t)(((int32_t)raw * 100) / 128);
}

/**
  * @brief  通过 I2C1 读取 MLX90393 三轴磁场数据。
  * @note   先发送启动 ZYX 测量命令，再等待转换完成并读取 X/Y/Z 原始值。
  * @retval X/Y/Z 三轴原始值平均值；如需真实物理单位，需按 MLX90393 配置增益另行换算。
  */
static int16_t Read_MLX90393_Mag(void)
{
  uint8_t cmd;
  uint8_t data[7] = {0};
  int16_t mx;
  int16_t my;
  int16_t mz;

  cmd = MLX90393_CMD_START_ZYX;
  if (HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, &cmd, 1, 100) != HAL_OK)
  {
    return 0;
  }

  HAL_Delay(20);
  cmd = MLX90393_CMD_READ_ZYX;
  if (HAL_I2C_Master_Transmit(&hi2c1, MLX90393_ADDR, &cmd, 1, 100) != HAL_OK)
  {
    return 0;
  }
  if (HAL_I2C_Master_Receive(&hi2c1, MLX90393_ADDR, data, sizeof(data), 100) != HAL_OK)
  {
    return 0;
  }

  mx = (int16_t)((uint16_t)data[1] << 8 | data[2]);
  my = (int16_t)((uint16_t)data[3] << 8 | data[4]);
  mz = (int16_t)((uint16_t)data[5] << 8 | data[6]);

  return (int16_t)(((int32_t)mx + (int32_t)my + (int32_t)mz) / 3);
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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  gpio_init();
  DIO1_Init();
  /* 默认关闭 PA8 PMOS，切断 LoRa、TMP117、MLX90393 等外设电源，降低休眠功耗。 */
  HAL_GPIO_WritePin(TMP_CTRL_GPIO_Port, TMP_CTRL_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t payload[80];
    uint16_t volt_mv;
    int16_t temp_x100;
    int16_t mag;
    int len;

    /* 每次被 RTC 唤醒后先打开 PA8 PMOS，给传感器和 LLCC68 上电。 */
    HAL_GPIO_WritePin(TMP_CTRL_GPIO_Port, TMP_CTRL_Pin, GPIO_PIN_SET);
    /* 等待外设电源稳定。 */
    HAL_Delay(20);

    /* LoRa 模块可能刚重新上电，因此每轮发送前重新初始化 GPIO、DIO1 和 LLCC68。 */
    gpio_init();
    DIO1_Init();
    LoraInit();

    /* 依次采集电压、温度、磁场。 */
    volt_mv = Read_Voltage_mV();
    temp_x100 = Read_TMP117_Temp_x100();
    mag = Read_MLX90393_Mag();

    /* 按 readme 要求拼接发送字符串：ID、TEMP、Volt、MAG。 */
    len = snprintf((char *)payload, sizeof(payload), "ID:%04u,TEMP:%d.%02d,Volt:%umV,MAG:%d.",
                   (unsigned int)DEVICE_ID,
                   (int)(temp_x100 / 100),
                   (int)((temp_x100 < 0) ? -(temp_x100 % 100) : (temp_x100 % 100)),
                   (unsigned int)volt_mv,
                   (int)mag);

    if (len > 0)
    {
      /* LoraDataSend 内部会先执行 CAD/LBT 信道侦听，确认空闲后才真正发射。 */
      LoraDataSend(payload, (uint8_t)((len < (int)sizeof(payload)) ? len : (int)(sizeof(payload) - 1U)));
      /* 等待 TX_DONE 中断，期间轮询处理 LLCC68 DIO1 事件；超时后也继续进入低功耗。 */
      for (uint32_t wait_start = HAL_GetTick(); (uint32_t)(HAL_GetTick() - wait_start) < 2000U; )
      {
        DX_Lora_RadioIrqProcess();
      }
      /* 让 LLCC68 进入 sleep，减少 PA8 关闭前的额外消耗。 */
      sx126x_set_sleep(NULL, SX126X_SLEEP_CFG_COLD_START);
    }

    /* 关闭 PMOS，切断外设电源。 */
    HAL_GPIO_WritePin(TMP_CTRL_GPIO_Port, TMP_CTRL_Pin, GPIO_PIN_RESET);
    /* 配置 RTC 唤醒，然后 MCU 进入 STOP 低功耗模式（睡眠时间 5 秒）。 */
    Enter_Stop(5);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00B07CB4;
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RXEN_Pin|TXEN_Pin|TMP_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRST_LORA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pins : RXEN_Pin TXEN_Pin TMP_CTRL_Pin */
  GPIO_InitStruct.Pin = RXEN_Pin|TXEN_Pin|TMP_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRST_LORA_Pin */
  GPIO_InitStruct.Pin = NRST_LORA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 LLCC68 NSS */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO2_Pin */
  GPIO_InitStruct.Pin = DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/* USER CODE BEGIN 4 */

static void Enter_Stop(uint32_t sleep_time_s)
{
  /* 重新配置 RTC WakeUp 前，先关闭上一次唤醒定时器。 */
  (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  /* 清除唤醒标志，避免旧标志导致无法正常进入 STOP。 */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  /* RTC CK_SPRE 为 1Hz，计数值为 sleep_time_s - 1 时约 sleep_time_s 秒唤醒一次。 */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, sleep_time_s - 1U, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }

  /* 进入 STOP 前暂停 SysTick，防止 SysTick 周期中断把 MCU 立即唤醒。 */
  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  /* RTC 唤醒后恢复 SysTick，并重新配置系统时钟。 */
  HAL_ResumeTick();

  SystemClock_Config();
  /* 唤醒后关闭本轮 wakeup timer，下一次休眠前会重新配置。 */
  (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
}

/**
  * @brief RTC WakeUp 中断回调。
  * @note  当前只用于唤醒 STOP 模式，无需额外处理。
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc_arg)
{
  (void)hrtc_arg;
}

/* USER CODE END 4 */

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

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
#include "nami_protocol.h"
#include "socket.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int dir;
    float vmax;
    uint32_t rise_time_ms;
    uint32_t slip_time_ms;
    uint32_t pause_time_ms;
    int32_t steps;
    uint8_t curve_percent;
} GeneratorConfig_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define V_REF 3.3f
#define RAMP_BUFFER_SIZE 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
const NaMi_Device_Info g_device_info = {
    .device_id        = "stm32_dac_001",
    .type             = "dac",
    .version          = "1.0_hw",
    .software_version = "1.0_fw",
    .tcp_port         = 6000
};

const NaMi_Network_Config g_network_config = {
    .mac = {0x02, 0x08, 0xDC, 0xAA, 0xBB, 0xCC},
    .ip  = {192, 168, 0, 123},
    .subnet_mask = {255, 255, 255, 0},
    .gateway = {192, 168, 0, 1}
};

const NaMi_Hardware_Config g_hardware_config = {
    .spi_handle = &hspi1,
    .cs_port    = W5500_CS_GPIO_Port,
    .cs_pin     = W5500_CS_Pin,
    .rst_port   = W5500_RST_GPIO_Port,
    .rst_pin    = W5500_RST_Pin
};

GeneratorConfig_t config;
uint32_t dac_buffer[RAMP_BUFFER_SIZE];
volatile int32_t steps_remaining = 0;
volatile uint8_t generator_running = 0;
volatile uint8_t pause_active = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void handle_device_commands(cJSON *command, char* session_id);
void send_json_response(const char* status, const char* message);
void send_json_event(const char* event_type, const char* message);

void init_default_config(void);
void generate_stick_curve(void);
void start_generator(void);
void stop_generator(void);
void start_single_cycle(void);
void start_pause(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef hlpuart1;

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

void init_default_config(void) {
    config.dir = 1;
    config.vmax = 2.5f;
    config.rise_time_ms = 200;
    config.slip_time_ms = 50;
    config.pause_time_ms = 100;
    config.steps = 0;
    config.curve_percent = 0;
}

void generate_stick_curve(void) {
    uint32_t v_max_dac = (uint32_t)((config.vmax / V_REF) * 4095.0f);
    if (v_max_dac > 4095) v_max_dac = 4095;

    uint32_t total_time = config.rise_time_ms + config.slip_time_ms;
    if (total_time == 0) total_time = 1;
    uint32_t rise_points = (config.rise_time_ms * RAMP_BUFFER_SIZE) / total_time;
    if (rise_points < 2) rise_points = 2;
    if (rise_points > RAMP_BUFFER_SIZE - 2) rise_points = RAMP_BUFFER_SIZE - 2;
    uint32_t slip_points = RAMP_BUFFER_SIZE - rise_points;

    float curve_factor = 1.0f + (config.curve_percent / 100.0f) * 4.0f;

    for (uint32_t i = 0; i < rise_points; i++) {
        float progress = (float)i / (float)(rise_points - 1);
        float curve_value = (config.dir == 1)
            ? pow(progress, curve_factor) * (float)v_max_dac
            : (float)v_max_dac * (1.0f - pow(progress, curve_factor));
        dac_buffer[i] = (uint32_t)(fminf(4095.0f, fmaxf(0.0f, curve_value)) + 0.5f);
    }

    uint32_t last_rise_value = dac_buffer[rise_points - 1];
    for (uint32_t i = 0; i < slip_points; i++) {
        float progress = (float)i / (float)(slip_points - 1);
        float slip_value_f = (float)last_rise_value - ((float)last_rise_value * progress);
        dac_buffer[rise_points + i] = (uint32_t)(fminf(4095.0f, fmaxf(0.0f, slip_value_f)) + 0.5f);
    }
    dac_buffer[RAMP_BUFFER_SIZE - 1] = 0;
}

void start_generator() {
    stop_generator();
    generate_stick_curve();

    uint32_t total_time_ms = config.rise_time_ms + config.slip_time_ms;
    if (total_time_ms == 0) total_time_ms = 1;
    uint32_t timer_period_us = (total_time_ms * 1000) / RAMP_BUFFER_SIZE;
    if (timer_period_us == 0) timer_period_us = 1;
    __HAL_TIM_SET_AUTORELOAD(&htim6, timer_period_us - 1);

    steps_remaining = config.steps;
    generator_running = 1;
    pause_active = 0;
    start_single_cycle();
    printf("INFO: Generator started.\n");
}

void stop_generator() {
    if (generator_running) {
        printf("INFO: Generator stopped.\n");
    }
    generator_running = 0;
    pause_active = 0;
    HAL_TIM_Base_Stop(&htim6);
    HAL_TIM_Base_Stop_IT(&htim7);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
}

void start_single_cycle(void) {
    if (!generator_running) return;
    pause_active = 0;
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    HAL_TIM_Base_Start(&htim6);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_buffer, RAMP_BUFFER_SIZE, DAC_ALIGN_12B_R);
}

void start_pause(void) {
    if (!generator_running) return;
    if (config.pause_time_ms == 0) {
        start_single_cycle();
        return;
    }
    pause_active = 1;
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);

    uint32_t prescaler = (HAL_RCC_GetPCLK1Freq() / 1000) - 1;
    htim7.Instance->PSC = prescaler;
    uint32_t pause_value = config.pause_time_ms;
    if (pause_value > 65535) pause_value = 65535;
    if (pause_value == 0) pause_value = 1;
    __HAL_TIM_SET_AUTORELOAD(&htim7, pause_value - 1);
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    HAL_TIM_Base_Start_IT(&htim7);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    if (!generator_running) return;
    HAL_TIM_Base_Stop(&htim6);

    if (steps_remaining > 0) {
        steps_remaining--;
        if (steps_remaining == 0 && config.steps != 0) {
            stop_generator();
            printf("EVENT: Generator finished steps.\n");
            send_json_event("generator_finished_steps", "All steps completed.");
            return;
        }
    }
    start_pause();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        HAL_TIM_Base_Stop_IT(&htim7);
        if (!generator_running || !pause_active) return;
        start_single_cycle();
    }
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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  printf("--- NaMi DAC Generator ---\r\n");
  init_default_config();

  if (NaMi_Init(&g_hardware_config, &g_network_config, &g_device_info, handle_device_commands) != 0) {
        printf("FATAL: NaMi Protocol Init failed.\r\n");
        Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  NaMi_Process();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 19;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 19999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : W5500_RST_Pin */
  GPIO_InitStruct.Pin = W5500_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W5500_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_INT_Pin */
  GPIO_InitStruct.Pin = W5500_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(W5500_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_CS_Pin */
  GPIO_InitStruct.Pin = W5500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W5500_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void send_json_response(const char* status, const char* message) {
    cJSON *resp_json = cJSON_CreateObject();
    if (resp_json == NULL) {
        printf("ERROR: cJSON_CreateObject failed.\n");
        return;
    }
    cJSON_AddStringToObject(resp_json, "status", status);
    cJSON_AddStringToObject(resp_json, "message", message);

    char* response_string = cJSON_PrintUnformatted(resp_json);
    if (response_string) {
        send(1, (uint8_t*)response_string, strlen(response_string));
        free(response_string);
    }
    cJSON_Delete(resp_json);
}

void send_json_event(const char* event_type, const char* message) {
    cJSON *event_json = cJSON_CreateObject();
    if (event_json == NULL) {
        printf("ERROR: cJSON_CreateObject for event failed.\n");
        return;
    }
    cJSON_AddStringToObject(event_json, "event", event_type);
    cJSON_AddStringToObject(event_json, "message", message);

    char* event_string = cJSON_PrintUnformatted(event_json);
    if (event_string) {
        send(1, (uint8_t*)event_string, strlen(event_string));
        free(event_string);
    }
    cJSON_Delete(event_json);
}

void handle_device_commands(cJSON *command, char* session_id) {
    cJSON *cmd_item = cJSON_GetObjectItem(command, "cmd");
    if (!cJSON_IsString(cmd_item)) return;

    char* cmd_string = cmd_item->valuestring;
    printf("CMD: Received command '%s'\n", cmd_string);

    if (strcmp(cmd_string, "stop") == 0) {
        stop_generator();
        send_json_response("OK", "Generator stopped");
    }
    else if (strcmp(cmd_string, "config") == 0) {
        cJSON *params = cJSON_GetObjectItem(command, "params");
        if (cJSON_IsObject(params)) {
            cJSON *dir_item = cJSON_GetObjectItem(params, "dir");
            cJSON *vmax_item = cJSON_GetObjectItem(params, "vmax");
            cJSON *rise_time_item = cJSON_GetObjectItem(params, "rise_time_ms");
            cJSON *slip_time_item = cJSON_GetObjectItem(params, "slip_time_ms");
            cJSON *pause_time_item = cJSON_GetObjectItem(params, "pause_time_ms");
            cJSON *curve_item = cJSON_GetObjectItem(params, "curve_percent");
            cJSON *steps_item = cJSON_GetObjectItem(params, "steps");

            if (cJSON_IsNumber(dir_item) && cJSON_IsNumber(vmax_item) && cJSON_IsNumber(rise_time_item) &&
                cJSON_IsNumber(slip_time_item) && cJSON_IsNumber(pause_time_item) && cJSON_IsNumber(curve_item) &&
                cJSON_IsNumber(steps_item))
            {
                config.dir = dir_item->valueint;
                config.vmax = vmax_item->valuedouble;
                config.rise_time_ms = rise_time_item->valueint;
                config.slip_time_ms = slip_time_item->valueint;
                config.pause_time_ms = pause_time_item->valueint;
                config.curve_percent = curve_item->valueint;
                config.steps = steps_item->valueint;

                printf("CONFIG: New config applied.\n");
                start_generator();
                send_json_response("OK", "Generator configured and started");
            } else {
                printf("ERROR: Invalid or missing parameters in 'config' command.\n");
                send_json_response("ERROR", "Invalid or missing parameters");
            }
        } else {
            printf("ERROR: 'config' command requires a 'params' object.\n");
            send_json_response("ERROR", "Missing 'params' object");
        }
    } else {
        printf("WARN: Unknown command '%s'\n", cmd_string);
        send_json_response("ERROR", "Unknown command");
    }
}
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

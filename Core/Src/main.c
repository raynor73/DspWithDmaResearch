/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "arm_math.h"
#include "fir.h"
#include "fir_coeffs_361Taps_44100_200_3000.h"
#include "liquidcrystal_i2c.h"
#include "si5351.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum AdcDmaState {
	FILLING_FIRST_HALF, FILLING_SECOND_HALF
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_SIZE 4096
#define FULL_BUFFER_SIZE 8192
#define SAMPLE_RATE 44100
#define MAX_VALUE_FROM_ADC 0x0FFF
#define NUMBER_OF_ADC_CHANNELS 2

#define FREQUENCY_STEP_VARIANTS 4
#define MIN_FREQUENCY 5800000
#define MAX_FREQUENCY 7600000

#define ENCODER_SW_PIN GPIOG, GPIO_PIN_12
#define DEBOUNCE_DELAY 50 // milliseconds

#define SI5351_CORRECTION 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static arm_fir_instance_f32 g_filterFir;
static float32_t g_filterFirState[FILTER_TAP_NUM + DATA_SIZE - 1];

static arm_fir_instance_f32 g_delayFir;
static float32_t g_delayFirState[HILBERT_AND_DELAY_FILTERS_TAP_NUM + DATA_SIZE - 1];

static arm_fir_instance_f32 g_hilbertFir;
static float32_t g_hilbertFirState[HILBERT_AND_DELAY_FILTERS_TAP_NUM + DATA_SIZE - 1];

static uint32_t g_adcBuffer[NUMBER_OF_ADC_CHANNELS * FULL_BUFFER_SIZE];

static float32_t g_iBuffer[DATA_SIZE];
static float32_t g_qBuffer[DATA_SIZE];

static uint32_t g_dacBuffer[FULL_BUFFER_SIZE];

static float32_t g_inputBuffer[DATA_SIZE];
static float32_t g_outputBuffer[DATA_SIZE];

static volatile enum AdcDmaState g_adcDmaState = FILLING_FIRST_HALF;
static volatile int g_isDspPerformed = 0;

static int g_frequencyStepIndex = 2;
static long g_frequencySteps[FREQUENCY_STEP_VARIANTS] = { 10, 100, 1000, 10000 };
static char *g_frequencyStepNames[FREQUENCY_STEP_VARIANTS] = { "10 Hz ", "100 Hz", "1 kHz ", "10 kHz" };

static uint32_t g_frequency = 7200000;

static int g_lastEncoderSwitchLevel;
static unsigned long g_lastEncoderSwitchLevelChangeTimestamp;

static int16_t g_lastEncoderCounter = 0;
static int g_shouldApplyFrequency = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	g_adcDmaState = FILLING_SECOND_HALF;
	g_isDspPerformed = 0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	g_adcDmaState = FILLING_FIRST_HALF;
	g_isDspPerformed = 0;
}

static void renderUI()
{
	char string[17];

	HD44780_SetCursor(0, 0);
	memset(string, 0, sizeof(string));
	sprintf(string, "Freq: %lu Hz", g_frequency);
	HD44780_PrintStr(string);

	HD44780_SetCursor(0, 1);
	memset(string, 0, sizeof(string));
	sprintf(string, "Step: %s", g_frequencyStepNames[g_frequencyStepIndex]);
	HD44780_PrintStr(string);
}

static void updateFrequencyStep() {
	g_frequencyStepIndex++;
	if (g_frequencyStepIndex >= FREQUENCY_STEP_VARIANTS) {
		g_frequencyStepIndex = 0;
	}
}

static void increaseFrequency() {
	g_frequency += g_frequencySteps[g_frequencyStepIndex];
	if (g_frequency > MAX_FREQUENCY) {
		g_frequency = MAX_FREQUENCY;
	}
}

static void decreaseFrequency() {
	g_frequency -= g_frequencySteps[g_frequencyStepIndex];
	if (g_frequency < MIN_FREQUENCY) {
		g_frequency = MIN_FREQUENCY;
	}
}

static void applyFrequency(uint32_t frequency) {
	si5351PLLConfig_t si5351PllConfig;
	si5351OutputConfig_t si5351outputConfig;

	si5351_CalcIQ(g_frequency, &si5351PllConfig, &si5351outputConfig);

	uint8_t phaseOffset = (uint8_t) si5351outputConfig.div;
	si5351_SetupOutput(0, SI5351_PLL_A, SI5351_DRIVE_STRENGTH_4MA, &si5351outputConfig, 0);
	si5351_SetupOutput(2, SI5351_PLL_A, SI5351_DRIVE_STRENGTH_4MA, &si5351outputConfig, phaseOffset);
	si5351_SetupPLL(SI5351_PLL_A, &si5351PllConfig);
}

static void processInputAndUpdateUiIfNecessary() {
	int encoderSwitchLevel = HAL_GPIO_ReadPin(ENCODER_SW_PIN);
	unsigned long currentTimestamp = HAL_GetTick();

	if (encoderSwitchLevel != g_lastEncoderSwitchLevel && (currentTimestamp - g_lastEncoderSwitchLevelChangeTimestamp) > DEBOUNCE_DELAY) {
		g_lastEncoderSwitchLevel = encoderSwitchLevel;
		g_lastEncoderSwitchLevelChangeTimestamp = currentTimestamp;
		if (encoderSwitchLevel == GPIO_PIN_SET) {
			updateFrequencyStep();
			renderUI();
		}
	}

	if (g_shouldApplyFrequency) {
		g_shouldApplyFrequency = 0;
		applyFrequency(g_frequency);
		renderUI();
	}
}

static void performDsp()
{
	if (!g_isDspPerformed) {
		g_isDspPerformed = 1;

		uint32_t currentDacIndex = __HAL_DMA_GET_COUNTER(hdac1.DMA_Handle1);

		switch (g_adcDmaState) {
		case FILLING_FIRST_HALF:
			for (int i = 0; i < DATA_SIZE; i++) {
				g_iBuffer[i] = g_adcBuffer[NUMBER_OF_ADC_CHANNELS * DATA_SIZE + NUMBER_OF_ADC_CHANNELS * i];
				g_qBuffer[i] = g_adcBuffer[NUMBER_OF_ADC_CHANNELS * DATA_SIZE + NUMBER_OF_ADC_CHANNELS * i + 1];
			}
			break;

		case FILLING_SECOND_HALF:
			for (int i = 0; i < DATA_SIZE; i++) {
				g_iBuffer[i] = g_adcBuffer[NUMBER_OF_ADC_CHANNELS * i];
				g_qBuffer[i] = g_adcBuffer[NUMBER_OF_ADC_CHANNELS * i + 1];
			}
			break;
		}

		/*for (int i = 0; i < DATA_SIZE; i++) {
			g_inputBuffer[i] = g_iBuffer[i];
		}*/

		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_SET) {
			//arm_fir_f32(&g_delayFir, g_inputBuffer, g_outputBuffer, DATA_SIZE);

			arm_fir_f32(&g_delayFir, g_iBuffer, g_inputBuffer, DATA_SIZE);
			arm_fir_f32(&g_hilbertFir, g_qBuffer, g_iBuffer, DATA_SIZE);
			arm_add_f32(g_inputBuffer, g_iBuffer, g_qBuffer, DATA_SIZE);

			/*for (int i = 0; i < DATA_SIZE; i++) {
				g_inputBuffer[i] = g_qBuffer[i] + g_outputBuffer[i];
			}*/

			arm_fir_f32(&g_filterFir, g_qBuffer, g_outputBuffer, DATA_SIZE);

			for (int i = 0; i < DATA_SIZE; i++) {
				g_outputBuffer[i] += MAX_VALUE_FROM_ADC / 2;
			}

			/*for (int i = 0; i < DATA_SIZE; i++) {
				g_outputBuffer[i] = g_qBuffer[i];
			}*/
		} else {
			for (int i = 0; i < DATA_SIZE; i++) {
				g_outputBuffer[i] = g_qBuffer[i];
			}
		}



		for (int i = 0; i < DATA_SIZE; i++) {
			int dstDacIndex = currentDacIndex + DATA_SIZE + i;
			if (dstDacIndex >= FULL_BUFFER_SIZE) {
				dstDacIndex -= FULL_BUFFER_SIZE;
			}
			g_dacBuffer[dstDacIndex] = g_outputBuffer[i];
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	int16_t encoderCounter = (int16_t) __HAL_TIM_GET_COUNTER(htim);

	if (encoderCounter > g_lastEncoderCounter) {
		decreaseFrequency();

		g_lastEncoderCounter = encoderCounter;

		g_shouldApplyFrequency = 1;
	} else if (encoderCounter < g_lastEncoderCounter) {
		increaseFrequency();

		g_lastEncoderCounter = encoderCounter;

		g_shouldApplyFrequency = 1;
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
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_ETH_Init();
  MX_TIM6_Init();
  MX_ADC3_Init();
  MX_DAC1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc3, g_adcBuffer, NUMBER_OF_ADC_CHANNELS * FULL_BUFFER_SIZE);
  HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, g_dacBuffer, FULL_BUFFER_SIZE, DAC_ALIGN_12B_R);

  //delay_taps[84] = 1; // initialize delay FIR taps

  arm_fir_init_f32(&g_filterFir, FILTER_TAP_NUM, filter_taps, g_filterFirState, DATA_SIZE);
  arm_fir_init_f32(&g_delayFir, HILBERT_AND_DELAY_FILTERS_TAP_NUM, coeffs_delay_361, g_delayFirState, DATA_SIZE);
  arm_fir_init_f32(&g_hilbertFir, HILBERT_AND_DELAY_FILTERS_TAP_NUM, coeffs_hilbert_361Taps_44100_200_3000, g_hilbertFirState, DATA_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  si5351_Init(SI5351_CORRECTION);

  si5351PLLConfig_t si5351PllConfig;
  si5351OutputConfig_t si5351outputConfig;

  si5351_CalcIQ(g_frequency, &si5351PllConfig, &si5351outputConfig);
  /*
   * `phaseOffset` is a 7bit value, calculated from Fpll, Fclk and desired phase shift.
   * To get N° phase shift the value should be round( (N/360)*(4*Fpll/Fclk) )
   * Two channels should use the same PLL to make it work. There are other restrictions.
   * Please see AN619 for more details.
   *
   * si5351_CalcIQ() chooses PLL and MS parameters so that:
   *   Fclk in [1.4..100] MHz
   *   out_conf.div in [9..127]
   *   out_conf.num = 0
   *   out_conf.denum = 1
   *   Fpll = out_conf.div * Fclk.
   * This automatically gives 90° phase shift between two channels if you pass
   * 0 and out_conf.div as a phaseOffset for these channels.
   */
  uint8_t phaseOffset = (uint8_t) si5351outputConfig.div;
  si5351_SetupOutput(0, SI5351_PLL_A, SI5351_DRIVE_STRENGTH_4MA, &si5351outputConfig, 0);
  si5351_SetupOutput(2, SI5351_PLL_A, SI5351_DRIVE_STRENGTH_4MA, &si5351outputConfig, phaseOffset);

  /*
   * The order is important! Setup the channels first, then setup the PLL.
   * Alternatively you could reset the PLL after setting up PLL and channels.
   * However since _SetupPLL() always resets the PLL this would only cause
   * sending extra I2C commands.
   */
  si5351_SetupPLL(SI5351_PLL_A, &si5351PllConfig);
  si5351_EnableOutputs((1<<0) | (1<<2));

  g_lastEncoderSwitchLevel = HAL_GPIO_ReadPin(ENCODER_SW_PIN);
  g_lastEncoderSwitchLevelChangeTimestamp = HAL_GetTick();

  HD44780_Init(2);
  HD44780_Clear();

  renderUI();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  processInputAndUpdateUiIfNecessary();
	  performDsp();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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

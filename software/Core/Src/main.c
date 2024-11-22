/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "led_functions.h"

// Follow these steps: https://community.st.com/t5/stm32-mcus/configuring-dsp-libraries-on-stm32cubeide/ta-p/49637
#include "arm_math.h"
#include "arm_const_structs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROWS 8		// Number of cathode channels
#define COLUMNS 11	// Number of anode channels

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim21;
TIM_HandleTypeDef htim22;

/* USER CODE BEGIN PV */

uint8_t DEBUG_MODE = 0; // When set, enables debug LED and USB output

uint32_t adc_val;
uint32_t prev_adc_val;
uint8_t sample_flag = 0;	// Set when ADC needs to be sampled
uint8_t high_flag = 0;		// Set when heart beat has hit peak (high)
uint8_t low_flag = 1;		// Set when heart beat has hit low/valley
uint16_t sample_count = 0;	// Store number of samples between beats
int16_t heartbeat_freq = 30; 	// In BPM (signed to use in FIR filter)
#define DEFAULT_BPM 69

// WINDOW AVERAGE HEARTRATE VALUES
//uint16_t heartrate_window_val

// FIR filter for main signal
#define SAMPLE_FREQ 100
#define NUM_TAPS 150
q15_t Filt_Coeffs[NUM_TAPS] = {29, 20, 9, -4, -19, -36, -55, -75, -95, -116, -136, -155, -173, -189, -202, -212, -218, -221, -220, -215, -206, -194, -179, -161, -142, -122, -102, -84, -67, -53, -44, -39, -41, -48, -63, -85, -114, -150, -192, -241, -294, -350, -408, -467, -525, -579, -628, -669, -702, -724, -733, -729, -709, -675, -624, -557, -474, -376, -265, -141, -7, 135, 283, 434, 585, 733, 875, 1009, 1131, 1239, 1331, 1404, 1458, 1490, 1501, 1490, 1458, 1404, 1331, 1239, 1131, 1009, 875, 733, 585, 434, 283, 135, -7, -141, -265, -376, -474, -557, -624, -675, -709, -729, -733, -724, -702, -669, -628, -579, -525, -467, -408, -350, -294, -241, -192, -150, -114, -85, -63, -48, -41, -39, -44, -53, -67, -84, -102, -122, -142, -161, -179, -194, -206, -215, -220, -221, -218, -212, -202, -189, -173, -155, -136, -116, -95, -75, -55, -36, -19, -4, 9, 20, 29, 0};
q15_t Filt_State[NUM_TAPS] = {0};
arm_fir_instance_q15 S_FIR;

// FIR filter to find zero crossings (avg of current signal)
#define NUM_ZEROS_TAPS 512
q15_t Zeros_Filt_Coeffs[NUM_ZEROS_TAPS] = {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64};
q15_t Zeros_Filt_State[NUM_ZEROS_TAPS] = {0};
arm_fir_instance_q15 S_ZEROS;

// FIR filter to smooth heart rate estimations
#define NUM_BPM_TAPS 8
q15_t BPM_Filt_Coeffs[NUM_BPM_TAPS] = {4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096};
q15_t BPM_Filt_State[NUM_BPM_TAPS] = {0};
arm_fir_instance_q15 S_BPM;

// AUTO-CALIBRATING MIN/MAX
#define WINDOW_LEN 256			// Length of window to consider for min/max (smaller is quicker)
#define WINDOW_LEN_PWR 8		// Used for bit shifting instead of dividing
#define FFT_MULTIPLIER 4		// Define how much zero padding you want
#define FFT_MULTIPLIER_PWR 2	// Used for bit shifting instead of dividing
#define FFT_LEN (WINDOW_LEN*FFT_MULTIPLIER)
#define FFT_LEN_PWR (WINDOW_LEN_PWR+FFT_MULTIPLIER_PWR)	// Used for bit shifting instead of dividing

int16_t window_values[WINDOW_LEN] = {0};

uint16_t window_index = 0;
uint16_t calibration_count = 0;
uint16_t calibration_high = 2500;	// Value used in low to high comparison
uint16_t calibration_low = 2400;	// Value used in high to low comparison
uint16_t window_max = 0;
uint16_t window_min = 0;


uint8_t pattern_flag = 0; 	// Set when pattern handler needs called
uint8_t button_flag = 0; 	// Set when button is pressed
uint8_t button_prev = GPIO_PIN_SET;	// Previous state of button
uint8_t button_state;
uint8_t pattern_select = 0;
uint8_t led_timer_lock_flag = 0;	// When set, don't Handle LED queue in interrupt

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM21_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM22_Init(void);
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

	extern TIM_HandleTypeDef *cathode_timers[ROWS];
	cathode_timers[0] = &htim2;
	cathode_timers[1] = &htim2;
	cathode_timers[2] = &htim2;
	cathode_timers[3] = &htim2;
	cathode_timers[4] = &htim3;
	cathode_timers[5] = &htim3;
	cathode_timers[6] = &htim3;
	cathode_timers[7] = &htim3;


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
	MX_USB_DEVICE_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM21_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_TIM22_Init();
	/* USER CODE BEGIN 2 */

	// CHECK FOR BUTT1 PRESS, BUTT1 PRESS AT BOOT MEANS DEBUG MODE
	if (HAL_GPIO_ReadPin(BUTT1_GPIO_Port, BUTT1_Pin) == GPIO_PIN_RESET) {
		DEBUG_MODE = 1;
	}

	// Set LED timer periods/duty cycle
	//TIM3->CCR1 = 99;
	TIM2->CCR1 = 20;

	// Timer to control green LED
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	// Timer to control LED queue
	HAL_TIM_Base_Start_IT(&htim22);

	// Timer to control LED patterns
	HAL_TIM_Base_Start_IT(&htim21);
	//HAL_TIM_OC_Start_IT(&htim21, TIM_CHANNEL_1);

	// Timer to control button debouncing
	HAL_TIM_Base_Start_IT(&htim6);


	// Set ADC timer configuration to achieve desired SAMPLE_FREQ
	uint32_t core_freq = HAL_RCC_GetHCLKFreq();
	uint16_t adc_timer_prescaler = 200-1;	// Picked arbitrarily
	uint16_t adc_timer_period = (core_freq / (adc_timer_prescaler+1) / SAMPLE_FREQ ) - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim7, adc_timer_period);
	__HAL_TIM_SET_PRESCALER(&htim7, adc_timer_prescaler);
	// Timer to control ADC sampling
	HAL_TIM_Base_Start_IT(&htim7);

	// Calibrate ADC
	HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

	// Set up FIR filters
	arm_fir_init_q15(&S_FIR, NUM_TAPS, Filt_Coeffs, Filt_State, 1);
	arm_fir_init_q15(&S_ZEROS, NUM_ZEROS_TAPS, Zeros_Filt_Coeffs, Zeros_Filt_State, 1);
	arm_fir_init_q15(&S_BPM, NUM_BPM_TAPS, BPM_Filt_Coeffs, BPM_Filt_State, 1);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//HandleLEDQueue(); // Now handled in timer interrupt for asynchronous
		if (pattern_flag) {
			led_timer_lock_flag = 1;
			switch (pattern_select) {
			case 0:
				PulseHandler();			// Measured heartbeat
				break;
			case 1:
				PulseHandlerKeepOn();	// Measured heartbeat
				break;
			case 2:
				PrintRate(heartbeat_freq);	// Measured heartbeat
				break;
			case 3:
				PulseHandler();			// Constant heartbeat
				break;
			case 4:
				PulseHandlerKeepOn();	// Constant heartbeat
				break;
			}
			pattern_flag = 0;
			led_timer_lock_flag = 0;
		}

		if (button_flag) {
			pattern_select++;
			led_timer_lock_flag = 1;
			ResetIndexes();
			led_timer_lock_flag = 0;
			if (pattern_select > 4) {
				pattern_select = 0;							// Reset patterns
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// Turn on green LED
			}
			else if (pattern_select > 2) {
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);	// Turn off green LED
				TIM21->ARR = 869;							// Preset heartbeat to 69 bpm to avoid glitches
			}
			button_flag = 0;
		}

		if (sample_flag) {

			HAL_ADC_Start(&hadc);
			HAL_ADC_PollForConversion(&hadc, 100);
			adc_val = HAL_ADC_GetValue(&hadc);

			int16_t filter_input = (int16_t)adc_val - 2048;
			int16_t filtered_sample;
			int16_t filtered_avg;
			arm_fir_fast_q15(&S_FIR, &filter_input, &filtered_sample, 1);
			arm_fir_fast_q15(&S_ZEROS, &filtered_sample, &filtered_avg, 1);


			if (DEBUG_MODE) {
				// REMOVE USB STUFF TO REDUCE FLICKER
				char buf[32] = {0};
				sprintf(buf, "%d %d\r\n", (int16_t)filtered_sample, heartbeat_freq);
				if (heartbeat_freq >= 100) {
					CDC_Transmit_FS((uint8_t*)&buf, 10);
				}
				else {
					CDC_Transmit_FS((uint8_t*)&buf, 9);
				}
			}

			window_values[window_index] = adc_val;
			// TODO:
			//window_index++;
			prev_adc_val = adc_val;
			sample_flag = 0;

			sample_count++; // Increment sample count

			// If signal goes from low to high
			if (low_flag && filtered_sample > filtered_avg) {
				low_flag = 0;
				high_flag = 1;
				if (DEBUG_MODE) {
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				}
			}
			// If signal goes from high to low
			else if (high_flag && filtered_sample < filtered_avg) {
				low_flag = 1;
				high_flag = 0;
				if (DEBUG_MODE) {
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				}

				// Compute heart rate
				float heart_beat_float = (float)SAMPLE_FREQ / (float)sample_count * 60.0f;
				int16_t heartbeat_freq_raw = heart_beat_float;

				// If heartbeat measurement is within bounds, process. Otherwise, ignore
				if (heartbeat_freq_raw >= 40 && heartbeat_freq_raw <= 160) {
					// Moving avg filter heart rate
					arm_fir_fast_q15(&S_BPM, &heartbeat_freq_raw, &heartbeat_freq, 1);

					// Limit filtered measurement
					if (heartbeat_freq < 40) {
						heartbeat_freq = 40;
					}
					else if (heartbeat_freq > 160){
						heartbeat_freq = 160;
					}

					//  DIVIDE ARR BY NUM OF PATTERN STAGES
					//  16,000,000 Hz / 1600 / 20,0000 / 1 = 0.5 Hz
					//  16,000,000 Hz / 1600 / 3,000 / 1 = 3.333 Hz

					//	ARR = 16,000,000 / 1600 / (BPM/60)
					//	ARR = 16,000,000 / 1600 / BPM * 60 / Stages

					// When printing out numbers, just set constant timer frequency
					if (pattern_select == 2) {
						TIM21->ARR = 200;
					}
					// Default BPM for non sampling modes
					else if (pattern_select > 2) {
						TIM21->ARR = SystemCoreClock / (TIM21->PSC+1) / DEFAULT_BPM * 60 / pattern_steps[pattern_select] - 1;
					}
					// Adjust pattern to BPM
					else {
						// Range (2221 (30 BPM) to 332 (200 BPM)
						TIM21->ARR = SystemCoreClock / (TIM21->PSC+1) / heartbeat_freq * 60 / pattern_steps[pattern_select] - 1;
					}

				}
				sample_count = 0;	// Reset sample count

			}
		}

		// Process window once window is filled
		if (window_index == WINDOW_LEN) {
			// Compute average
			int16_t window_avg = 0;
			int32_t window_avg_sum = 0;
			for (uint16_t i =0; i < WINDOW_LEN; i++) {
				window_avg_sum += window_values[i];
			}
			window_avg = window_avg_sum >> WINDOW_LEN_PWR;	// Quick divide (assume length is power of 2)

			// Vectors for FFT processing
			q15_t window_real[FFT_LEN] = {0};
			q15_t window_complex[2*FFT_LEN] = {0};		// First two data points are X[0] (DC) and X[N/2] (Nyquist)
			q15_t window_fft_mag[FFT_LEN] = {0};

			// Set real data to be window + zero padding
			for (uint16_t i = 0; i < WINDOW_LEN; i++) {
				window_real[i] = (int16_t)window_values[i] - window_avg;
			}

			// Compute real FFT of data
			uint32_t ifftFlag = 0;
			uint32_t bitRevFlag = 1;
			arm_rfft_instance_q15 fft_instance;
			arm_rfft_init_q15(&fft_instance, FFT_LEN, ifftFlag, bitRevFlag);		// ifftFlag = 0 for regular (not inverse) FFT, bitRev = 1 for output to be in normal order
			arm_rfft_q15(&fft_instance, window_real, window_complex);
			arm_shift_q15(window_complex, FFT_LEN_PWR-1, window_complex, FFT_LEN);	// Bit shift 7 for FFT of length 256 per the documentation?
			arm_cmplx_mag_q15(window_complex, window_fft_mag, FFT_LEN);

			// Sampling time: (should be 25 Hz right now)
			uint16_t sample_freq = SystemCoreClock / (TIM7->PSC+1) / (TIM7->ARR+1);	// In Hz

			// Only look at FFT values in the range we're interested in
			uint16_t min_index = 30 * FFT_LEN / sample_freq / 60;
			uint16_t max_index = 199 * FFT_LEN / sample_freq / 60;

			// Find largest peak in first half of FFT (positive frequencies)
			int16_t max_peak = 0;
			uint16_t max_peak_index = 0;
			for (uint16_t i = min_index; i < max_index; i++) {
				if (window_fft_mag[i] >= max_peak) {
					max_peak = window_fft_mag[i];
					max_peak_index = i;
				}
			}

			// Compute heartbeat from FFT index
			float heart_beat_float = (float)max_peak_index / (float)FFT_LEN * (float)sample_freq*60.0;
			heartbeat_freq = (uint16_t)heart_beat_float;

			/*
			// Print max FFT index
			if (DEBUG_MODE) {
				char buf[32] = {0};
				sprintf(buf, "FFT peak: %d\r\n", (uint16_t)heart_beat_float);
				CDC_Transmit_FS((uint8_t*)&buf, strlen(buf));
			}
			 */

			// Limit between 30 BPM and 200 BPM
			if (heartbeat_freq < 30) {
				heartbeat_freq = 30;
			}
			else if (heartbeat_freq > 199){
				heartbeat_freq = 199;
			}

			//  DIVIDE ARR BY NUM OF PATTERN STAGES
			//  16,000,000 Hz / 1600 / 20,0000 / 1 = 0.5 Hz
			//  16,000,000 Hz / 1600 / 3,000 / 1 = 3.333 Hz

			//	ARR = 16,000,000 / 1600 / (BPM/60)
			//	ARR = 16,000,000 / 1600 / BPM * 60 / Stages

			// When printing out numbers, just set constant timer frequency
			if (pattern_select == 2) {
				TIM21->ARR = 200;
			}
			// Default BPM for non sampling modes
			else if (pattern_select > 2) {
				TIM21->ARR = 869;
			}
			// Adjust pattern to BPM
			else {
				// Range (2221 (30 BPM) to 332 (200 BPM)
				TIM21->ARR = SystemCoreClock / (TIM21->PSC+1) / heartbeat_freq * 60 / pattern_steps[pattern_select] - 1;
			}

			window_index = 0;
		}	// End of processing window

		//HAL_Delay(1);

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
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
	hadc.Init.OversamplingMode = ENABLE;
	hadc.Init.Oversample.Ratio = ADC_OVERSAMPLING_RATIO_16;
	hadc.Init.Oversample.RightBitShift = ADC_RIGHTBITSHIFT_4;
	hadc.Init.Oversample.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
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
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 32-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 100-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 32-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 100-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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
	htim6.Init.Prescaler = 3200-1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 100-1;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
	htim7.Init.Prescaler = 200-1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 3200-1;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */

	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM21 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM21_Init(void)
{

	/* USER CODE BEGIN TIM21_Init 0 */

	/* USER CODE END TIM21_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM21_Init 1 */

	/* USER CODE END TIM21_Init 1 */
	htim21.Instance = TIM21;
	htim21.Init.Prescaler = 1600-1;
	htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim21.Init.Period = 1000-1;
	htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim21) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM21_Init 2 */

	/* USER CODE END TIM21_Init 2 */

}

/**
 * @brief TIM22 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM22_Init(void)
{

	/* USER CODE BEGIN TIM22_Init 0 */

	/* USER CODE END TIM22_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM22_Init 1 */

	/* USER CODE END TIM22_Init 1 */
	htim22.Instance = TIM22;
	htim22.Init.Prescaler = 160-1;
	htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim22.Init.Period = 20-1;
	htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim22, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim22) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM22_Init 2 */

	/* USER CODE END TIM22_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, C0_Pin|C1_Pin|C2_Pin|C3_Pin
			|C4_Pin|C5_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, C6_Pin|C7_Pin|LED1_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, A0_Pin|A8_Pin|A9_Pin|A10_Pin
			|A1_Pin|A2_Pin|A3_Pin|A4_Pin
			|A5_Pin|A6_Pin|A7_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : C0_Pin C1_Pin C2_Pin C3_Pin
                           C4_Pin C5_Pin */
	GPIO_InitStruct.Pin = C0_Pin|C1_Pin|C2_Pin|C3_Pin
			|C4_Pin|C5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : C6_Pin C7_Pin A0_Pin A8_Pin
                           A9_Pin A10_Pin LED1_Pin A1_Pin
                           A2_Pin A3_Pin A4_Pin A5_Pin
                           A6_Pin A7_Pin */
	GPIO_InitStruct.Pin = C6_Pin|C7_Pin|A0_Pin|A8_Pin
			|A9_Pin|A10_Pin|LED1_Pin|A1_Pin
			|A2_Pin|A3_Pin|A4_Pin|A5_Pin
			|A6_Pin|A7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTT1_Pin */
	GPIO_InitStruct.Pin = BUTT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTT1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Check if timer to handle LED flashing
	if (htim == &htim22 && led_timer_lock_flag != 1) {
		// NOTE, I AM DANGEROUSLY ALLOWING INTERRUPT TO CALL MULTIPLE TIMES FOR OPTIMAL SPEED
		// TOOK SOME GUESSING TO GET THIS TO WORK
		HandleLEDQueue();
	}

	// Check if timer to control pattern steps
	else if (htim == &htim21)
	{
		//PulseHandler();
		pattern_flag = 1;
		//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

	}
	// Check if timer to sample button press
	else if (htim == &htim6) {
		button_state = HAL_GPIO_ReadPin(BUTT1_GPIO_Port, BUTT1_Pin);
		if (button_state == GPIO_PIN_RESET && button_prev == GPIO_PIN_SET) {
			button_flag = 1;
			button_prev = GPIO_PIN_RESET;
		}
		else if (button_state == GPIO_PIN_SET && button_prev == GPIO_PIN_RESET) {
			button_prev = GPIO_PIN_SET;
		}

	}
	// Check if timer to sample ADC
	else if (htim == &htim7) {
		sample_flag = 1;
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
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "ucpd.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max30003.h"
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

const int EINT_STATUS_MASK =  1 << 23;
const int BINT_STATUS_MASK = 0x080000;
const int RRINT_STATUS_MASK = 0x000400;
const int FIFO_OVF_MASK =  0x7;
const int FIFO_VALID_SAMPLE_MASK =  0x0;
const int FIFO_FAST_SAMPLE_MASK =  0x1;
const int FIFO_RANGE_SAMPLE_MASK = 0x1;
const int ETAG_BITS_MASK = 0x7;
const int BTAG_BITS_MASK = 0x7;


#define NUM_STAGES_2ORDER 1 // Number of biquad stages
#define NUM_STAGES_4ORDER 2 // Number of biquad stages (for a 4th order filter)
#define BLOCK_SIZE 48// Number of samples to process at a time
#define SAMPLE_RATE 32
#define PAST_SAMPLE_BUFFER (SAMPLE_RATE * 3) //sample rate * n, n decides the second in samples to look back at


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t ecgFIFO, readECGSamples, idx, ETAG[32], status;
uint32_t biozFIFO, readBIOZSamples, BTAG[8];
int32_t ecgSample[32], biozSample[8];
int ecgFIFOIntFlag;
uint32_t rtor_interval_int;
float rtor_interval;


float32_t phasicCoeffs[NUM_STAGES_2ORDER*5] = {
    // Stage 1

		0.998266003962435,	-1.99653200792487,	0.998266003962435,	1.99652900118035,	-0.996535014669388
};
float32_t tonicCoeffs[NUM_STAGES_2ORDER*5] = {
    // Stage 1
		6.00307975164863e-06,	1.20061595032973e-05,	6.00307975164863e-06,	1.99305802314321,	-0.993082035462212
};
float32_t filteredCoeffs[NUM_STAGES_2ORDER*5] = {
    // Stage 1
		0.00490303497078511,	0.00980606994157022	,0.00490303497078511,	1.79238563711149,	-0.811997776994632
};
/* State buffer for the biquad filter */

static float32_t phasicState[NUM_STAGES_2ORDER*2];
static float32_t tonicState[NUM_STAGES_2ORDER*2];
static float32_t filteredState[NUM_STAGES_2ORDER*2];

static arm_biquad_cascade_df2T_instance_f32 S_phasic;
static arm_biquad_cascade_df2T_instance_f32 S_tonic;
static arm_biquad_cascade_df2T_instance_f32 S_filtered;

float32_t outputphasic[BLOCK_SIZE];
float32_t outputtonic[BLOCK_SIZE];
float32_t outputfiltered[BLOCK_SIZE];

float32_t outputphasicMiddle[BLOCK_SIZE];
float32_t outputtonicMiddle[BLOCK_SIZE];
float32_t outputfilteredMiddle[BLOCK_SIZE];



uint32_t n_sample = 0;
float32_t conductanceSample[BLOCK_SIZE];
float32_t reverseSample[BLOCK_SIZE];
uint32_t skip_sample = 0;


float32_t phasic_window[PAST_SAMPLE_BUFFER];

static  float32_t previous_1 = 0;
static  float32_t previous_2 = 0;
uint8_t valid_peak = 0;
float32_t peak[BLOCK_SIZE];
float32_t onset[BLOCK_SIZE];

uint8_t SCR_rise_sample = 0;
float32_t SCR_rise_time = 0;
float32_t mean_risetime = 0;
float32_t old_mean_risetime = 0;
static uint32_t count_mean_risetime= 0;
float32_t std_dev_risetime = 0;


float32_t mean_tonic = 0;
static uint32_t count_mean_tonic = 0;
static uint16_t settling_samples_tonic = 0;

float32_t max_tonic = 0;
float32_t max_phasic = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

void Falling_callback(){
	ecgFIFOIntFlag = 1;
}

void Printing(){
}

void StdDev_RiseTime(float32_t time){
	count_mean_risetime++;
	mean_risetime = mean_risetime + (time-mean_risetime)/count_mean_risetime;
	std_dev_risetime = std_dev_risetime + (time - mean_risetime) * (time - old_mean_risetime);
	old_mean_risetime = mean_risetime;

//	printf("BIOZ_STD_RISETIME:%.20f,", std_dev_risetime);
}

void Local_Peak_and_Onset(float32_t signal,float32_t *peak, float32_t *onset) {
	SCR_rise_time = 0;
	if(valid_peak > 0){
	    		valid_peak--;
	    	}

    if (previous_1 > previous_2 && previous_1 > signal) {
    	previous_2 = previous_1;
    	previous_1 = signal;



    	float32_t lowest_point = phasic_window[1];
    	SCR_rise_sample = 1;
		for (uint8_t i = 1; i < PAST_SAMPLE_BUFFER-valid_peak; i++) {
			if (phasic_window[i] < lowest_point) {
				SCR_rise_sample = i;
				lowest_point = phasic_window[i];
			}
		}
		if (previous_2 - lowest_point > 4e-7){
			*onset = lowest_point;
			*peak = previous_2;
			valid_peak = PAST_SAMPLE_BUFFER - 1;
			SCR_rise_time = SCR_rise_sample;
			StdDev_RiseTime(SCR_rise_time);
		}
		else{
			*onset = 0;
			*peak = 0;


		}
    }
    else{
    	previous_2 = previous_1;
		previous_1 = signal;

		*onset = 0;
		*peak = 0;
    }
}

void Phasic_window(float32_t phasic){

	for (uint8_t i = PAST_SAMPLE_BUFFER; i > 0; i--) {
	        phasic_window[i - 1] = phasic_window[ i - 2];
	    }

	phasic_window[0] = phasic;

}

void Mean_Tonic(float32_t tonic){

	if(settling_samples_tonic > 32*10){
		count_mean_tonic++;
		mean_tonic = mean_tonic + (tonic-mean_tonic)/count_mean_tonic;
	}
	else{
		settling_samples_tonic++;
	}
}

void Max_Signal(float32_t signal, float32_t* max){
	if (*max < signal){
		*max = signal;
	}
}

void flip(float32_t Array[], uint32_t dimArray){
	uint32_t nforward=0;
	uint32_t nbackward=dimArray-1;
	float32_t temp;
	 while(nforward<nbackward){
	      //swap
	      temp = Array[nforward];
	      Array[nforward] = Array[nbackward];
	      Array[nbackward] = temp;

	      nforward++;
	      nbackward--;
	    }
}

void filtfilt(const arm_biquad_cascade_df2T_instance_f32 * S,
  float32_t * pInput,
  float32_t * pMiddle,
  float32_t * pOutput,
  uint32_t blockSize)
{
	arm_biquad_cascade_df2T_f32(S, pInput, pMiddle, blockSize);

	flip(pMiddle, blockSize);

	arm_biquad_cascade_df2T_f32(S, pMiddle, pOutput, blockSize);

	flip(pOutput, blockSize);

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

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_UCPD1_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  LL_SPI_Enable(SPI1);
  LL_SPI_StartMasterTransfer(SPI1);
  max30003Begin();
  //max30003Config();
  //max30003BeginRtorMode(); 	//Configures and enable ECG/RTOR
  //max30001gBeginBIOZ(); 		//Configures and enable BIOZ

  max30001gBeginBothMode4Electrode();
//  max30003ReadInfo();
//  max30003Printconf();

  arm_biquad_cascade_df2T_init_f32(&S_phasic, NUM_STAGES_2ORDER, phasicCoeffs, phasicState);
  arm_biquad_cascade_df2T_init_f32(&S_tonic, NUM_STAGES_2ORDER, tonicCoeffs, tonicState);
  arm_biquad_cascade_df2T_init_f32(&S_filtered, NUM_STAGES_2ORDER, filteredCoeffs, filteredState);
  uint8_t buffer_in[4];


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if( ecgFIFOIntFlag ) {


	  		  ecgFIFOIntFlag = 0;
	  		  DATA_Read(buffer_in, STATUS, 3);
	  		  status = buff8_to_int32(buffer_in);
	  		  // Check if EINT interrupt asserted

	  		  if (( status & EINT_STATUS_MASK) == EINT_STATUS_MASK) {
	  			  readECGSamples = 0;                        					 // Reset sample counter

	  			  do {

	  				  DATA_Read(buffer_in, ECG_FIFO, 3);
	  				  ecgFIFO = buff8_to_int32(buffer_in);  					 // Read FIFO
	  				  ecgSample[readECGSamples] = (ecgFIFO >> 6);                // Isolate voltage data
	  				  ecgSample[readECGSamples] = CorrectSign(ecgSample[readECGSamples], 18);
	  				  ETAG[readECGSamples] = ( ecgFIFO >> 3 ) & ETAG_BITS_MASK;  // Isolate ETAG
	  				  readECGSamples++;                                          // Increment sample counter

	  				  // Check that sample is not last sample in FIFO
	  			  } while ( ETAG[readECGSamples-1] == FIFO_VALID_SAMPLE_MASK ||
	  					  ETAG[readECGSamples-1] == FIFO_FAST_SAMPLE_MASK );

	  			  // Check if FIFO has overflowed
	  			  if( ETAG[readECGSamples - 1] == FIFO_OVF_MASK ){
	  				  //printf("OVERFLOW");
	  				  DATA_Write(FIFO_RST,3, 0x000000);   // Reset FIFO
	  				  //rLed = 1;//notifies the user that an over flow occured
	  			  }

	  			  // Print results
	  			  for( idx = 0; idx < readECGSamples; idx++ ) {
	  				  //printf("ECG:%ld,", ecgSample[idx]);

	  			  }
	  		  }
	  		  if((status & RRINT_STATUS_MASK ) == RRINT_STATUS_MASK ){

	  			  DATA_Read(buffer_in, RTOR, 3);
	  			  rtor_interval_int = buff8_to_int32(buffer_in);
	  			  rtor_interval_int = (rtor_interval_int >> 10);
	  			  rtor_interval = rtor_interval_int * 0.008;
	  			  //printf("RTOR:%f,", rtor_interval);
	  		  }

	  		  if (( status & BINT_STATUS_MASK) == BINT_STATUS_MASK) {
	  			  			  readBIOZSamples = 0;                        					// Reset sample counter

				  do {
					  DATA_Read(buffer_in, BIOZ_FIFO, 3);
					  biozFIFO = buff8_to_int32(buffer_in);  					  // Read FIFO
					  biozSample[readBIOZSamples] = biozFIFO >> 4;                // Isolate voltage data
					  biozSample[readBIOZSamples] = CorrectSign(biozSample[readBIOZSamples], 20);
					  BTAG[readBIOZSamples] = (biozFIFO) & BTAG_BITS_MASK;        // Isolate BTAG
					  readBIOZSamples++;                                          // Increment sample counter
					  // Check that sample is not last sample in FIFO
				  } while ( BTAG[readBIOZSamples-1] == FIFO_VALID_SAMPLE_MASK ||
						  BTAG[readBIOZSamples-1] == FIFO_RANGE_SAMPLE_MASK );

				  // Check if FIFO has overflowed
				  if( BTAG[readBIOZSamples - 1] == FIFO_OVF_MASK ){
					  //printf("OVERFLOW");
					  DATA_Write(FIFO_RST,3, 0x000000);   // Reset FIFO

					  //rLed = 1;//notifies the user that an over flow occured
				  }

				  // Print results
				  for( idx = 0; idx < readBIOZSamples; idx++ ) {
					  if (skip_sample > 63){
						  conductanceSample[n_sample] = ((pow(2, 19) * 10 * 110 * pow(10, -9))) / biozSample[idx];
						  reverseSample[n_sample] = ((pow(2, 19) * 10 * 110 * pow(10, -9))) / biozSample[7-idx];
						  if (n_sample == BLOCK_SIZE - 1){
//							  arm_biquad_cascade_df2T_f32(&S_filtered, conductanceSample, outputfiltered, BLOCK_SIZE);
							  arm_biquad_cascade_df2T_f32(&S_filtered, conductanceSample, outputfiltered, BLOCK_SIZE);
							  arm_biquad_cascade_df2T_f32(&S_phasic, outputfiltered, outputphasic, BLOCK_SIZE);
							  filtfilt(&S_tonic, conductanceSample, outputtonicMiddle, outputtonic, BLOCK_SIZE);

							  for (uint8_t i = 0; i < BLOCK_SIZE; i++) {
								  Phasic_window(outputphasic[i]);
								  Local_Peak_and_Onset(outputphasic[i], &peak[i], &onset[i]);
								  Mean_Tonic(outputtonic[i]);
								  Max_Signal(outputtonic[i], &max_tonic);
								  Max_Signal(outputphasic[i], &max_phasic);
//								  printf("BIOZ:%.14f,", conductanceSample[i]);
//								  printf("BIOZ_FILTERED:%.4f,", outputfiltered[i]);
//								  printf("BIOZ_TONIC:%.14f,", outputtonic[i]);
//								  printf("BIOZ_PHASIC:%.14f,", outputphasic[i]);
//								  printf("BIOZ_PEAK:%.14f,", peak[i]);
//								  printf("BIOZ_ONSET:%.14f,", onset[i]);
//								  printf("BIOZ_SCR_RISE_TIME:%.4f,", SCR_rise_time);
//								  printf("BIOZ_TONIC_MEAN:%.14f,", mean_tonic);

								  HAL_UART_Transmit(&huart1, (uint8_t*)&conductanceSample[i], 4, HAL_MAX_DELAY);
								  HAL_UART_Transmit(&huart1, (uint8_t*)&outputfiltered[i], 4, HAL_MAX_DELAY);
								  HAL_UART_Transmit(&huart1, (uint8_t*)&outputtonic[i], 4, HAL_MAX_DELAY);
								  HAL_UART_Transmit(&huart1, (uint8_t*)&outputphasic[i], 4, HAL_MAX_DELAY);
								  HAL_UART_Transmit(&huart1, (uint8_t*)&peak[i], 4, HAL_MAX_DELAY);

							  }
							  n_sample = 0;
						  }
						  else{
							  n_sample++;
						  }

					  }
					  else{
						  skip_sample++;
					  }


//					  HAL_UART_Transmit(&huart1, (uint8_t*)&filteredBiozSample, 4, HAL_MAX_DELAY);
//					  HAL_UART_Transmit(&huart1, (uint8_t*)&biozSample[idx], 4, HAL_MAX_DELAY);


				  }
//				  DATA_Read(buffer_in, BIOZ_FIFO, 3);
//				  biozFIFO = buff8_to_int32(buffer_in);  					  // Read FIFO
//				  biozSample[0] = biozFIFO >> 4;                // Isolate voltage data
//				  biozSample[0] = CorrectSign(biozSample[0], 20);
//				  BTAG[0] = (biozFIFO) & BTAG_BITS_MASK;        // Isolate BTAG
//
//				  if( BTAG[0] == FIFO_OVF_MASK ){
//					  //printf("OVERFLOW");
//					  DATA_Write(FIFO_RST,3, 0x000000);   // Reset FIFO
//
//					  //rLed = 1;//notifies the user that an over flow occured
//				  }
//				  else{
//					  HAL_UART_Transmit(&huart1, (uint8_t*)&biozSample[0], 4, HAL_MAX_DELAY);
//				  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
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

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
#define BLOCK_SIZE 16// Number of samples to process at a time



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

int32_t complete_signal[1920];

float32_t phasicCoeffs[NUM_STAGES_2ORDER*5] = {
    // Stage 1
		0.996535014651354,	-1.99307002930271,	0.996535014651354,	1.99305802314321,	-0.993082035462212
};
float32_t tonicCoeffs[NUM_STAGES_4ORDER*5] = {
    // Stage 1
		3.60560142312659e-11,	7.21120284625319e-11,	3.60560142312659e-11,	1.99094683136613,	-0.990970818249545,
		1,	2,	1,	1.99622602297120,	-0.996250073458198
};
float32_t filteredCoeffs[NUM_STAGES_4ORDER*5] = {
    // Stage 1
		2.4419565e-05,	4.8863920e-05,	2.4419731e-05,	1.7422240,	-0.76127446,
		1,	2,	1,	1.8731294,	-0.89363289
};
/* State buffer for the biquad filter */

static float32_t phasicState[NUM_STAGES_2ORDER*4];
static float32_t tonicState[NUM_STAGES_4ORDER*4];
static float32_t filteredState[NUM_STAGES_4ORDER*4];

static arm_biquad_casd_df1_inst_f32 S_phasic;
static arm_biquad_casd_df1_inst_f32 S_tonic;
static arm_biquad_casd_df1_inst_f32 S_filtered;

float32_t outputphasic[BLOCK_SIZE];
float32_t outputtonic[BLOCK_SIZE];
float32_t outputfiltered[BLOCK_SIZE];

float32_t outputphasicForward[BLOCK_SIZE];
float32_t outputphasicReverse[BLOCK_SIZE];
float32_t outputtonicForward[BLOCK_SIZE];
float32_t outputtonicReverse[BLOCK_SIZE];


uint32_t n_sample = 0;
float32_t conductanceSample[BLOCK_SIZE];
float32_t reverseSample[BLOCK_SIZE];
uint32_t skip_sample = 0;

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
	printf("yooooo\n");
}

float32_t ButterworthFilter(float32_t input1) {
    // Coefficients obtained from MATLAB or any other design tool for a single biquad section
	double b[] = {2.44195652983833e-05, 9.76782611935331e-05, 0.000146517391790300, 9.76782611935331e-05, 2.44195652983833e-05};
	double a[] = {1, -3.61535270201620, 4.91831571913911, -2.98287134586922, 0.680299041791089};


    // State variables (initialized to zero)
    static double x[4] = {0}; // Input delays
    static double y[4] = {0}; // Output delays

    // Apply the biquad section to the input
    double in = (double)input1;

    // Calculate the output for the biquad
    double out = b[0] * in + b[1] * x[0] + b[2] * x[1] + b[3] * x[2] + b[4] * x[3]
                - a[1] * y[0] - a[2] * y[1] - a[3] * y[2] - a[4] * y[3];

    // Update state variables
    for (int i = 3; i > 0; i--) {
        x[i] = x[i - 1];
        y[i] = y[i - 1];
    }

    x[0] = in;
    y[0] = out;

    return (float32_t)out;
}

float32_t ButterworthFilterTonic(float32_t input2) {
    float32_t b_tonic[] = {1.5033722e-06,	3.0067445e-06,	1.5033722e-06};
    float32_t a_tonic[] = {1, -1.9965290,	0.99653500};

    // State variables (initialized to zero)
    static float32_t x_2[3] = {0}; // Input delays
    static float32_t y_2[3] = {0}; // Output delays

    // Apply the biquad section
    float32_t in_2 = (float32_t)input2;
    float32_t out_2 = b_tonic[0] * in_2 + b_tonic[1] * x_2[0] + b_tonic[2] * x_2[1]
                    - a_tonic[1] * y_2[0] - a_tonic[2] * y_2[1];

    // Update state variables
    x_2[2] = x_2[1];
    x_2[1] = x_2[0];
    x_2[0] = in_2;

    y_2[2] = y_2[1];
    y_2[1] = y_2[0];
    y_2[0] = out_2;

    return out_2;
}
//
//
float32_t ButterworthFilterPhasic(float32_t input3) {
    float32_t b_phasic[] = {0.99826598,	-1.9965320,	0.99826598};
    float32_t a_phasic[] = {1,	-1.9965290,	0.99653500};

    // State variables (initialized to zero)
    static float32_t x_3[2] = {0}; // Input delays
    static float32_t y_3[2] = {0}; // Output delays

    // Apply the biquad sections (two cascaded second-order filters)
    float32_t in_3 = (float32_t)input3;

//    float32_t out_3 = b_phasic[0] * in_3 + b_phasic[1] * x_3[0] + b_phasic[2] * x_3[1] + b_phasic[3] * x_3[2] + b_phasic[4] * x_3[3]
//                    - a_phasic[1] * y_3[0] - a_phasic[2] * y_3[1] - a_phasic[3] * y_3[2] - a_phasic[4] * y_3[3];
    float32_t out_3 = b_phasic[0] * in_3 + b_phasic[1] * x_3[0] + b_phasic[2] * x_3[1]
                        - a_phasic[1] * y_3[0] - a_phasic[2] * y_3[1];
    // Update state variables
//    x_3[3] = x_3[2];
//    x_3[2] = x_3[1];
    x_3[1] = x_3[0];
    x_3[0] = in_3;

//    y_3[3] = y_3[2];
//    y_3[2] = y_3[1];
    y_3[1] = y_3[0];
    y_3[0] = out_3;

    return out_3;
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
  max30003ReadInfo();
  max30003Printconf();
  arm_biquad_cascade_df1_init_f32(&S_phasic, NUM_STAGES_2ORDER, phasicCoeffs, phasicState);
  arm_biquad_cascade_df1_init_f32(&S_tonic, NUM_STAGES_4ORDER, tonicCoeffs, tonicState);
  arm_biquad_cascade_df1_init_f32(&S_filtered, NUM_STAGES_4ORDER, filteredCoeffs, filteredState);
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
							  arm_biquad_cascade_df1_f32(&S_filtered, conductanceSample, outputfiltered, BLOCK_SIZE);

							  arm_biquad_cascade_df1_f32(&S_phasic, conductanceSample, outputphasicForward, BLOCK_SIZE);
							  arm_biquad_cascade_df1_f32(&S_phasic, reverseSample, outputphasicReverse, BLOCK_SIZE);

							  arm_biquad_cascade_df1_f32(&S_tonic, conductanceSample, outputtonicForward, BLOCK_SIZE);
							  arm_biquad_cascade_df1_f32(&S_tonic, reverseSample, outputtonicReverse, BLOCK_SIZE);

							  for (uint8_t i = 0; i < BLOCK_SIZE; i++) {
								  outputphasic[i] = 0.5 *(outputphasicForward[i] + outputphasicReverse[i]);
								  outputtonic[i] = 0.5* (outputtonicForward[i]+outputtonicReverse[i]);
									  printf("BIOZ_FILTERED:%.14f,", outputfiltered[i]);
									  printf("BIOZ:%.14f,", conductanceSample[i]);
									  printf("BIOZ_PHASIC:%.14f,", outputphasic[i]);
									  printf("BIOZ_TONIC:%.14f,", outputtonic[i]);
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

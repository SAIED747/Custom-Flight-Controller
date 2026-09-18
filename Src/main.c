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
//#define USE_HAL_DRIVER
#define ARM_MATH_CM7
#include <arm_math.h>
#include <stdio.h>
#include <stm32h7xx_hal_gpio.h>
#include "stm32h7xx_ll_tim.h"
//#include "mpu6050.h"
//#include "mpu9250.h"
//#include "bmi160.h"
//#include "qmc5883.h"
//#include "dps310.h"
#include "icm45686.h"
#include "stm32h7xx_it.h"
#include "stm32h7xx_hal_i2c.h"
#include "fs-ia6b.h"
//#include "m8n.h"
#include <axis3f.h>
#include "notchfilter.h"
#include "lpf2p.h"
#include "pid.h"
//#include <kalman_core.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern int16_t x_acc, y_acc, z_acc;
extern int16_t x_gyro, y_gyro, z_gyro;
int8_t data_ready_flag = 0;
uint32_t start, stop;
float usICM, usBMI, usDPS, usM8N, usFS, usQMC, usInitNotchArdu, usInitNotchPhils, usFFT;

////when working on the PID controller code.
/////////////////pitch conntroller//////////////////////////
extern float icm_pitch, icm_roll;
extern float ICM_GX, ICM_GY, ICM_GZ;
//float pitch_reference, pitch_error, pitch_p;
//float pitch_error_sum, pitch_i;
//float pitch_derivative, pitch_d;
//int OuterLoop_counter = 0;
//float pitch_out_kp, pitch_out_ki, pitch_out_kd, pitch_pid;
////pitch rate controller
//float pitch_rate_reference, pitch_rate_error, pitch_rate_p;
//float pitch_rate_error_sum, pitch_rate_i;
//float pitch_rate_derivative, pitch_rate_d;
//float pitch_in_kp, pitch_in_ki, pitch_in_kd, pitch_rate_pid;
////variables for the heavy LPF for the pitch rate used in the D-term of the inner loop PID
//float Gx_filt, Gx_filt_prev, alpha;
//
//////////////roll controller////////////////
//float roll_reference, roll_error, roll_p;
//float roll_error_sum, roll_i;
//float roll_derivative, roll_d;
////int OuterLoop_counter = 0;
//float roll_out_kp, roll_out_ki, roll_out_kd, roll_pid;
////roll rate controller
//float roll_rate_reference, roll_rate_error, roll_rate_p;
//float roll_rate_error_sum, roll_rate_i;
//float roll_rate_derivative, pitch_rate_d;
//float roll_in_kp, roll_in_ki, roll_in_kd, roll_rate_pid;
////variables for the heavy LPF for the roll rate used in the D-term of the inner loop PID
//float Gy_filt, Gy_filt_prev;


//notchfilter
NotchFilterFloat First_Harmonic_Notch_GX,  First_Harmonic_Notch_GY,  First_Harmonic_Notch_GZ;
NotchFilterFloat Second_Harmonic_Notch_GX, Second_Harmonic_Notch_GY, Second_Harmonic_Notch_GZ;
NotchFilterFloat Third_Harmonic_Notch_GX,  Third_Harmonic_Notch_GY,  Third_Harmonic_Notch_GZ;

//LPF2P
lpf2pData lpfData_GX, lpfData_GY, lpfData_GZ;



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
extern uint8_t uart1_rx_flag;
extern uint8_t uart1_rx_data;
extern uint8_t data_ready;
extern char rx_buffer[20];

extern uint8_t uart4_rx_flag;
extern uint8_t uart4_rx_data;

extern uint8_t uart5_rx_flag;
extern uint8_t uart5_rx_data;

extern uint8_t m8n_rx_buf[36];			//36-byte which is the size of 1-UBX-message frame
extern uint8_t m8n_rx_cplt_flag;		//flag to see that the recieving is completed

extern uint8_t ibus_rx_buf[100];
extern uint8_t ibus_rx_cplt_flag;

extern uint8_t tim2_2500us_flag;
extern uint8_t tim2_1s_flag;


//extern int lv;

//qmc5883p---magnetometer
extern uint8_t I2C_QMC5883P_DMA_data_ready_flag;
extern uint8_t QMC_MAG_rxData[6];


//icm45686
extern uint8_t ICM45686_INT_FLAG;

//BMI160
extern uint8_t BMI160_INT_FLAG;



//FFT variables
#define FFT_LENGTH 512
#define SAMPLING_RATE 1600

float32_t input_fft[FFT_LENGTH];
float32_t output_fft[FFT_LENGTH];
float32_t output_fft_mag[FFT_LENGTH / 2];

arm_rfft_fast_instance_f32 fft_instance;

float FFT_MAX_FREQ = 0;
float FFT_MAX_FREQ_MAG = 0;
float FFT_MAX_FREQ_PREV = 0;
//float FFT_MAG_VALUES = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int Is_iBus_Throttle_Min(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char* p, int len)
{
	for(int i=0; i<len; i++)
	{

		if (p[i] == '\n') {
		    while(!LL_USART_IsActiveFlag_TXE(USART1));
		       LL_USART_TransmitData8(USART1, '\r');   // add CR before LF
		}

		while(!LL_USART_IsActiveFlag_TXE(USART1));
		LL_USART_TransmitData8(USART1, *(p+i));

	}
	return len;
}

//Struct_BMI160 BMI160;
//Struct_DPS310 DPS310;
Struct_ICM45686 ICM45686;
//Struct_QMC5883P QMC5883P;


//kalmanCoreData_t this;
//kalmanCoreParams_t params;

Axis3f icm45686_gyro;
Axis3f icm45686_acc;
Axis3f_Bias gyro_bias, acc_bias;

Axis3f bmi160_gyro;
Axis3f bmi160_acc;

//int debugcounter1;
unsigned int ccr1, ccr2, ccr3, ccr4;




//float aaa=0;
//float32_t ab[3][2];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */




  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  HAL_Delay(3000);

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_I2C3_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  	LL_USART_EnableIT_RXNE_RXFNE(UART5);
  	LL_USART_EnableIT_RXNE_RXFNE(USART1);
  	LL_USART_EnableIT_RXNE_RXFNE(UART4);


  	//M8N_Initialization();
  	ICM45686_Initialization();
  	//BMI160_Initialization1();
  	//DPS310_Initialization();
  	//QMC5883P_Initialization(&hi2c3);
  	//MPU6050_Init(&hi2c1);
  	//MPU9250_Init(&MPU9250, dev,  accScale,  gyroScale,  magScale);

  	lpf2pInit(&lpfData_GX, 1600, 100);
  	lpf2pInit(&lpfData_GY, 1600, 100);
  	lpf2pInit(&lpfData_GZ, 1600, 100);



  	LL_TIM_EnableCounter(TIM2);
  	LL_TIM_EnableIT_UPDATE(TIM2);
  	LL_TIM_EnableCounter(TIM5);
  	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
  	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2);
  	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH3);
  	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH4);



	while(Is_iBus_Throttle_Min() == 0);

  	// Enable DWT CYCCNT
  	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  	DWT->CYCCNT = 0;
  	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


	//uint32_t nowMs = 0;
//  	int dt=0;
//	int quadisflying=0;
//	predictDt(&this, &params,&acc, &gyro, dt, quadisflying);
  	//kalmanCoreDefaultParams(&params);
  	//kalmanCoreInit(&this, &params, nowMs);





  	//arm_rfft_fast_init_f32(&fft_instance, FFT_LENGTH);
  	int fft_counter = 0;
  	//float ffff;



  	float kp=0, ki=0, kd=0;



  	pitch.out.kp = 1.5;
    pitch.out.ki = 0;
    pitch.out.kd = 0;
    pitch.in.kp  = 400;
    pitch.in.ki  = 250;
    pitch.in.kd  = 1;
    pitch.out.outer_loop_counter = 0;

    roll.out.kp  = 1.5;
    roll.out.ki  = 0;
    roll.out.kd  = 0;
    roll.in.kp   = 400;
    roll.in.ki   = 250;
    roll.in.kd   = 1;
    roll.out.outer_loop_counter = 0;

   // yaw.out.kp  = 1.1;
	//yaw.out.ki  = 0;
	//yaw.out.kd  = 0;
	yaw_rate.kp   = 100;
	yaw_rate.ki   = 20;
	yaw_rate.kd   = 0;
	//yaw.out.outer_loop_counter = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //start = DWT->CYCCNT;

	  if(tim2_1s_flag == 0)
	  {fft_counter = 0;}


	  if (data_ready)
	  {
	      data_ready = 0;
	      sscanf(rx_buffer, "%f,%f,%f", &kp, &ki, &kd);
	  }

//	  pitch_out_kp	=	1.1;
//	  pitch_out_ki	=	0;
//	  pitch_out_kd	=	0;
//
//	  pitch_in_kp	=	700;
//	  pitch_in_ki	=	300;
//	  pitch_in_kd	=	0;




	  // 400 Hz loop (every 0.0025s)
	  if(tim2_2500us_flag == 1)
	  {

		  tim2_2500us_flag = 0;
		  //OuterLoop_counter++;
		  // HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);

		  Double_Roll_Pitch_PID_Calculation(&pitch, (iBus.RV - 1500) * 0.05f, icm_pitch - 0.5, ICM_GX);
		  Double_Roll_Pitch_PID_Calculation(&roll, (iBus.RH - 1500) * 0.05f, icm_roll - 1.0, ICM_GY);

		  if(iBus.LV < 1030)
		  {
			  Reset_All_PID_Integrator();
		  }


		  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH - 1500) * 0.5f, ICM_GZ);

		  // 200 Hz loop (every 0.005s)
//		  if(OuterLoop_counter % 2 == 0)
//		  {
//
//
//			  pitch_reference = (iBus.RV - 1500) * 0.05f;	//mapping the RV stick to pitch target angle
//			  pitch_error = pitch_reference - icm_pitch;
//			  pitch_p = pitch_error * pitch_out_kp;
//
//			  pitch_error_sum = pitch_error_sum + pitch_error * 0.01;
//			  if(iBus.LV < 1030) pitch_error_sum = 0;
//			  pitch_i = pitch_error_sum * pitch_out_ki;
//
//			  pitch_derivative = ICM_GX;
//			  pitch_d = -pitch_derivative * pitch_out_kd;
//
//			  pitch_pid = pitch_p + pitch_i + pitch_d;
//		  }
//
//		  pitch_rate_reference = pitch_pid;//((iBus.RV - 1500) * 0.1f);//pitch_pid;//(((iBus.RV - 1500) * 0.1f) - icm_pitch) * 2;
//		  pitch_rate_error = pitch_rate_reference - ICM_GX;
//		  pitch_rate_p = pitch_rate_error * pitch_in_kp;
//
//
//		  pitch_rate_error_sum = pitch_rate_error_sum + pitch_rate_error * 0.0025;
//		  if(iBus.LV < 1030) pitch_rate_error_sum = 0;
//		  pitch_rate_i = pitch_rate_error_sum * pitch_in_ki;
//
//		  alpha = 0.24;	// Fc = 20hz, sampling at 400hz
//		  Gx_filt = (1 - alpha) * Gx_filt + alpha * ICM_GX;
//		  pitch_rate_derivative = (Gx_filt - Gx_filt_prev) / 0.0025;//if you want to use the D-term you should use heavy LPF only for the rate needed in
//		  Gx_filt_prev = Gx_filt;									//the D term, Fc = 20hz
//		  pitch_rate_d = -pitch_rate_derivative * pitch_in_kd;
//
//		  pitch_rate_pid = pitch_rate_p + pitch_rate_i + pitch_rate_d;

//		  ccr1 = 120000 + (iBus.LV - 1000) * 120 - pitch_rate_pid;
//		  ccr2 = 120000 + (iBus.LV - 1000) * 120 + pitch_rate_pid;
//		  ccr3 = 120000 + (iBus.LV - 1000) * 120 + pitch_rate_pid;
//		  ccr4 = 120000 + (iBus.LV - 1000) * 120 - pitch_rate_pid;

		  ccr1 = 120000 + (iBus.LV - 1000) * 120 - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result;
		  ccr2 = 120000 + (iBus.LV - 1000) * 120 + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
		  ccr3 = 120000 + (iBus.LV - 1000) * 120 + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result;
		  ccr4 = 120000 + (iBus.LV - 1000) * 120 - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;



	  }



	  //start = DWT->CYCCNT;
	  //BMI160
//	  if (BMI160_INT_FLAG)
//	  {
//		  start = DWT->CYCCNT;
//		  BMI160_Get6AxisRawData(&BMI160, &bmi160_gyro, &bmi160_acc);
//		  stop = DWT->CYCCNT;
//		  usBMI = (stop - start) / (SystemCoreClock / 1000000.0f); // convert to microseconds
//
//		  BMI160_INT_FLAG = 0;
//		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
//	  }

	  //ICM45686
	  //if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET)

	  if (ICM45686_INT_FLAG)
	  {
		//start = DWT->CYCCNT;
		ICM45686_Get3AxisGyroRawData(&ICM45686, &icm45686_gyro, &icm45686_acc, &gyro_bias);
		//stop = DWT->CYCCNT;
		//usICM = (stop - start) / (SystemCoreClock / 1000000.0f); // convert to microseconds


		//fft_counter is define directly under the initialization of fft
		if(tim2_1s_flag == 1)
		{
			if( fft_counter < FFT_LENGTH)
			{
				//if (ICM45686_INT_FLAG)
				//{
					//ICM45686_Get3AxisGyroRawData(&ICM45686, &icm45686_gyro, &icm45686_acc, &gyro_bias);
					input_fft[fft_counter] = ICM_GX;
					fft_counter++;
				//}
			}
			if(fft_counter == FFT_LENGTH)
			{
				fft_counter++;
				//if(iBus.SwC != 1500)
				//{
					//start = DWT->CYCCNT;
					arm_rfft_fast_f32(&fft_instance, input_fft, output_fft, 0);
					arm_cmplx_mag_f32(output_fft, output_fft_mag, FFT_LENGTH/2);
					//stop = DWT->CYCCNT;
					//usFFT = (stop - start) / (SystemCoreClock / 1000000.0f); // convert to microseconds
				//}
				FFT_MAX_FREQ_MAG = 0;
				for(int fft_counter1 = 1; fft_counter1 < FFT_LENGTH/2; fft_counter1++)
				{
					//if(iBus.SwC == 1500)
					//{
						//printf("%f\n", output_fft_mag[fft_counter1]);
						//FFT_MAG_VALUES = output_fft_mag[fft_counter1];
					//}
					if(output_fft_mag[fft_counter1] > FFT_MAX_FREQ_MAG)					//computing the the frequency with maximum magnitude to be notched(mototrs freq)
					{
						FFT_MAX_FREQ = fft_counter1 * ((float)SAMPLING_RATE / FFT_LENGTH);
						FFT_MAX_FREQ_MAG = output_fft_mag[fft_counter1];
					}
					if(FFT_MAX_FREQ > 60    ||  FFT_MAX_FREQ < 30)						//when the motors frequeuncy notched it is no more maximum
					{																	//so this conditions to preserve on the notch frequency to stay at motors freq
						FFT_MAX_FREQ = FFT_MAX_FREQ_PREV;
					}
					else
					{
						FFT_MAX_FREQ_PREV = FFT_MAX_FREQ;
					}
				}
					//start = DWT->CYCCNT;
//					NotchFilter_Init(&notchFilt, FFT_MAX_FREQ, 10, SAMPLING_RATE);
//					stop = DWT->CYCCNT;
//					usInitNotchPhils = (stop - start) / (SystemCoreClock / 1000000.0f); // convert to microseconds
					//start = DWT->CYCCNT;
					notch_init(&First_Harmonic_Notch_GX,  1600,  FFT_MAX_FREQ,  20,  40);
					notch_init(&First_Harmonic_Notch_GY,  1600,  FFT_MAX_FREQ,  20,  40);
					notch_init(&First_Harmonic_Notch_GZ,  1600,  FFT_MAX_FREQ,  20,  40);
					//stop = DWT->CYCCNT;
					//usInitNotchArdu = (stop - start) / (SystemCoreClock / 1000000.0f); // convert to microseconds
					notch_init(&Second_Harmonic_Notch_GX,  1600,  FFT_MAX_FREQ * 2,  20,  40);
					notch_init(&Second_Harmonic_Notch_GY,  1600,  FFT_MAX_FREQ * 2,  20,  40);
					notch_init(&Second_Harmonic_Notch_GZ,  1600,  FFT_MAX_FREQ * 2,  20,  40);

					notch_init(&Third_Harmonic_Notch_GX,  1600,  FFT_MAX_FREQ * 3,  20,  40);
					notch_init(&Third_Harmonic_Notch_GY,  1600,  FFT_MAX_FREQ * 3,  20,  40);
					notch_init(&Third_Harmonic_Notch_GZ,  1600,  FFT_MAX_FREQ * 3,  20,  40);
					//stop = DWT->CYCCNT;
					//usInitNotchArdu = (stop - start) / (SystemCoreClock / 1000000.0f); // convert to microseconds

				tim2_1s_flag = 0;

			}

		}








		ICM45686_INT_FLAG = 0;
	  	//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);


	  	//int dt=0;
	  	//int quadisflying=0;
	  	//predictDt(&this, &params,&acc, &gyro, dt, quadisflying);
	  }

	  //DPS310
	  //start = DWT->CYCCNT;///////////////////////////////////////////////////////////////////////////////////////////
	  //DPS310_GetTempPres(&DPS310);////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  //stop = DWT->CYCCNT;///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  //usDPS = (stop - start) / (SystemCoreClock / 1000000.0f); // convert to microseconds


	  //QCM5883P
//	  if(I2C_QMC5883P_DMA_data_ready_flag)
//	  {
//		  if (HAL_I2C_GetState(&hi2c3) == HAL_I2C_STATE_READY)
//		  {
//			  I2C_QMC5883P_DMA_data_ready_flag = 0;
//			  start = DWT->CYCCNT;
//			  QMC5883P_Get3AxisGyroRawData(&QMC5883P, QMC_MAG_rxData);
//			  stop = DWT->CYCCNT;
//			  usQMC = (stop - start) / (SystemCoreClock / 1000000.0f); // convert to microseconds
//		  }
//	  }

//	  //code for the gps
//	  if(m8n_rx_cplt_flag == 1)
//	  {
//		  m8n_rx_cplt_flag = 0;
//		  if(M8N_UBX_CHKSUM_Check(&m8n_rx_buf[0], 100) == 1)
//		  {
//			  //start = DWT->CYCCNT;
//			  M8N_UBX_NAV_PVT_Parsing(&m8n_rx_buf[0], &pvt);
//			  //stop = DWT->CYCCNT;
//			  //usM8N = (stop - start) / (SystemCoreClock / 1000000.0f); // convert to microseconds
//		  }
//	  }

	  //code for the ibus
	  if(ibus_rx_cplt_flag == 1)
	  {
		  ibus_rx_cplt_flag = 0;
		  if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		  {
			  //start = DWT->CYCCNT;
			  iBus_Parsing(&ibus_rx_buf[0], &iBus);
			  //stop = DWT->CYCCNT;
			  //usFS = (stop - start) / (SystemCoreClock / 1000000.0f); // convert to microseconds

			  if(iBus_isActiveFailsafe(&iBus) == 1)
			  {
				  //counter1--;	//if falesafe is happenning decreasment the counter by one
			  }
			  else
			  {
				  //counter1++;
			  }
		  }
	  }


	  if(iBus.LV > 1030)
	  {
		  TIM5->CCR1 = ccr1 > 240000 ? 240000 : ccr1 < 120000 ? 120000 : ccr1;
		  TIM5->CCR2 = ccr2 > 240000 ? 240000 : ccr2 < 120000 ? 120000 : ccr2;
		  TIM5->CCR3 = ccr3 > 240000 ? 240000 : ccr3 < 120000 ? 120000 : ccr3;
		  TIM5->CCR4 = ccr4 > 240000 ? 240000 : ccr4 < 120000 ? 120000 : ccr4;
	  }
	  else
	  {
		  TIM5->CCR1 = 120000;
		  TIM5->CCR2 = 120000;
		  TIM5->CCR3 = 120000;
		  TIM5->CCR4 = 120000;
	  }
//	  stop = DWT->CYCCNT;
//	  usBMI = (stop - start) / (SystemCoreClock / 1000000.0f); // convert to microseconds


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00200A26;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Enable Fast Mode Plus
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C3);
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 300000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 300000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM5, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM5);
  LL_TIM_SetClockSource(TIM5, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM5, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM5);
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  /**TIM5 GPIO Configuration
  PA0   ------> TIM5_CH1
  PA1   ------> TIM5_CH2
  PA2   ------> TIM5_CH3
  PA3   ------> TIM5_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  LL_USART_InitTypeDef UART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  /**UART4 GPIO Configuration
  PA11   ------> UART4_RX
  PA12   ------> UART4_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* UART4 interrupt Init */
  NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(UART4_IRQn);

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  UART_InitStruct.BaudRate = 9600;
  UART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  UART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  UART_InitStruct.Parity = LL_USART_PARITY_NONE;
  UART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  UART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  UART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;

  LL_USART_Init(UART4, &UART_InitStruct);
  LL_USART_DisableFIFO(UART4);
  LL_USART_SetTXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_ConfigAsyncMode(UART4);

  /* USER CODE BEGIN WKUPType UART4 */

  /* USER CODE END WKUPType UART4 */

  LL_USART_Enable(UART4);

  /* Polling UART4 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(UART4))) || (!(LL_USART_IsActiveFlag_REACK(UART4))))
  {
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  LL_USART_InitTypeDef UART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**UART5 GPIO Configuration
  PB12   ------> UART5_RX
  PB13   ------> UART5_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_14;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* UART5 interrupt Init */
  NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(UART5_IRQn);

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  UART_InitStruct.BaudRate = 115200;
  UART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  UART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  UART_InitStruct.Parity = LL_USART_PARITY_NONE;
  UART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  UART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  UART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;

  LL_USART_Init(UART5, &UART_InitStruct);
  LL_USART_DisableFIFO(UART5);
  LL_USART_SetTXFIFOThreshold(UART5, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(UART5, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_ConfigAsyncMode(UART5);

  /* USER CODE BEGIN WKUPType UART5 */

  /* USER CODE END WKUPType UART5 */

  LL_USART_Enable(UART5);

  /* Polling UART5 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(UART5))) || (!(LL_USART_IsActiveFlag_REACK(UART5))))
  {
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB14   ------> USART1_TX
  PB15   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE8 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB11 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int Is_iBus_Throttle_Min(void)
{
    if (ibus_rx_cplt_flag == 1)
    {
        ibus_rx_cplt_flag = 0;

        if (iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
        {
            iBus_Parsing(&ibus_rx_buf[0], &iBus);

            if (iBus.LV < 1010)
                return 1;
        }
    }


    return 0;
}


//__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
//
//  /* NOTE : This function should not be modified, when the callback is needed,
//            the HAL_UART_RxCpltCallback can be implemented in the user file.
//   */
//  //HAL_UART_Transmit(&huart1, rx_data, sizeof(rx_data), HAL_MAX_DELAY);
//}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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

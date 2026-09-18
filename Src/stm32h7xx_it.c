/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "icm45686.h"
#include "bmi160.h"
#include <m8n.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//These variables are global so they can be accessed by main and other file too.

extern uint8_t QMC_MAG_rxData[6];


uint8_t uart1_rx_flag = 0;
uint8_t uart1_rx_data = 0;

uint8_t uart4_rx_flag = 0;
uint8_t uart4_rx_data = 0;

uint8_t uart5_rx_flag = 0;
uint8_t uart5_rx_data = 0;

uint8_t m8n_rx_buf[100];			//36-byte which is the size of 1-UBX-message frame
uint8_t m8n_rx_cplt_flag = 0;	//flag to see that the recieving is completed

uint8_t ibus_rx_buf[32];			//32-byte which is the size of 1-iBus-message frame
uint8_t ibus_rx_cplt_flag = 0;	//flag to see that the recieving is completed


uint8_t tim2_2500us_flag = 0;
uint8_t tim2_10ms_flag = 0;
uint8_t tim2_1s_flag = 0;

uint8_t tim2_2500us_count = 0;
uint8_t tim2_10ms_count = 0;
uint16_t tim2_1s_count = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c3_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;
extern I2C_HandleTypeDef hi2c3;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c3_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c3_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */


	//HAL_Delay(500);

	if(LL_TIM_IsActiveFlag_UPDATE(TIM2))
	{
		LL_TIM_ClearFlag_UPDATE(TIM2);

		tim2_2500us_count++;
		if(tim2_2500us_count == 1)
		{
			tim2_2500us_count = 0;
			tim2_2500us_flag  = 1;
		}

		tim2_10ms_count++;
		if(tim2_10ms_count == 4)
		{
			tim2_10ms_count = 0;
			tim2_10ms_flag  = 1;
		}

		tim2_1s_count++;
		if(tim2_1s_count == 400)
		{
			tim2_1s_count = 0;
			tim2_1s_flag  = 1;
		}


	}

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
#define RX_BUFFER_SIZE 20

char rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
uint8_t data_ready = 0;
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if(LL_USART_IsActiveFlag_RXNE_RXFNE(USART1))
		{

			uart1_rx_data = LL_USART_ReceiveData8(USART1);
			//char c = LL_USART_ReceiveData8(USART1);

			if (uart1_rx_data == '\n')   // end of number
			{
				rx_buffer[rx_index] = '\0'; // terminate string
				rx_index = 0;
				data_ready = 1;
			}
			else
			{
				if (rx_index < RX_BUFFER_SIZE - 1)
				{
					rx_buffer[rx_index++] = uart1_rx_data;
				}
			}
		}
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	static unsigned char cnt = 0;		//static means to keep the value whenever the function is called(prevent initialization)

		if(LL_USART_IsActiveFlag_RXNE_RXFNE(UART4))
			{

				uart4_rx_data = LL_USART_ReceiveData8(UART4);
				uart4_rx_flag = 1;


				//this is the code for UBX recieving message
				switch(cnt)									//check the first 2 bytes if correct, store all the 36 bytes data in the temp buffer array
				{
				case 0:										//we are checking whether the first recieved data is 0xB5 and assigning is to the temp buffer
					if(uart4_rx_data == 0xb5)
					{
						m8n_rx_buf[cnt] = uart4_rx_data;
						cnt++;
					}
					break;

				case 1:										//same but for the second byte should be 0x62
					if(uart4_rx_data == 0x62)
					{
						m8n_rx_buf[cnt] = uart4_rx_data;
						cnt++;
					}
					else
						cnt = 0;
					break;

				case 2:										//same but for the second byte should be 0x62
					if(uart4_rx_data == 0x01)
					{
						m8n_rx_buf[cnt] = uart4_rx_data;
						cnt++;
					}
					else
						cnt = 0;
					break;

				case 3:										//same but for the second byte should be 0x62
					if(uart4_rx_data == 0x07)
					{
						m8n_rx_buf[cnt] = uart4_rx_data;
						cnt++;
					}
					else
						cnt = 0;
					break;


				case 99:
					m8n_rx_buf[cnt] = uart4_rx_data;
					cnt = 0;
					m8n_rx_cplt_flag = 1;					//to indicate that a message frame reception is complete
					break;

				default:
					m8n_rx_buf[cnt] = uart4_rx_data;
					cnt++;
					break;


				}
			}
  /* USER CODE END UART4_IRQn 0 */
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
	static unsigned char cnt = 0;		//static means to keep the value whenever the function is called(prevent initialization)


		if(LL_USART_IsActiveFlag_RXNE_RXFNE(UART5))
		{
			//counter++;

			uart5_rx_data = LL_USART_ReceiveData8(UART5);
			uart5_rx_flag = 1;


			switch(cnt)
			{
			case 0:
				if(uart5_rx_data == 0x20)
				{
					ibus_rx_buf[cnt] = uart5_rx_data;
					cnt++;
				}
				break;

			case 1:
				if(uart5_rx_data == 0x40)
				{
					ibus_rx_buf[cnt] = uart5_rx_data;
					cnt++;
				}
				else
					cnt = 0;
				break;


			case 31:
				ibus_rx_buf[cnt] = uart5_rx_data;
				cnt = 0;
				ibus_rx_cplt_flag = 1;
				break;

			default:
				ibus_rx_buf[cnt] = uart5_rx_data;
				cnt++;
				break;
			}

	//		while(!LL_USART_IsActiveFlag_TXE(USART1));
	//		LL_USART_TransmitData8(USART1, uart5_rx_data);
		}
  /* USER CODE END UART5_IRQn 0 */
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles I2C3 event interrupt.
  */
void I2C3_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_EV_IRQn 0 */

  /* USER CODE END I2C3_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_EV_IRQn 1 */

  /* USER CODE END I2C3_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C3 error interrupt.
  */
void I2C3_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_ER_IRQn 0 */

  /* USER CODE END I2C3_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_ER_IRQn 1 */

  /* USER CODE END I2C3_ER_IRQn 1 */
}

/* USER CODE BEGIN 1 */

//////////////////////////////////DMA//////////////////////////////
//QMC5883P-Magnwtometer
volatile uint8_t I2C_QMC5883P_DMA_data_ready_flag =0;
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C3)
	{
		I2C_QMC5883P_DMA_data_ready_flag = 1;
	}
}

//ICM45686
volatile uint8_t ICM45686_txData[13];
volatile uint8_t ICM45686_rxData[13];
volatile uint8_t ICM45686_INT_FLAG = 0;

//BMI160
volatile uint8_t BMI160_txData[13];
volatile uint8_t BMI160_rxData[13];
volatile uint8_t BMI160_INT_FLAG = 0;
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	//ICM45686
	if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY)
	{
		if (hspi == &hspi1)
		{
			ICM45686_INT_FLAG = 1;
			ICM45686_CS_HIGH();
		}
	}

	//BMI160
	if (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_READY)
	{
		if (hspi == &hspi2)
		{
			BMI160_INT_FLAG = 1;
			BMI160_CS_HIGH();
		}
	}
}

/////////////////////////////////INTERRUPT/////////////////////////

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == GPIO_PIN_9)
	{
		if (HAL_I2C_GetState(&hi2c3) == HAL_I2C_STATE_READY)
		{
			HAL_I2C_Mem_Read_DMA(&hi2c3, 0x2C<<1  , 0x01, I2C_MEMADD_SIZE_8BIT, QMC_MAG_rxData, 6);
		}

	}


	if(ICM45686_INT_FLAG == 0)
	{
		if (GPIO_Pin == GPIO_PIN_0)
		{
			ICM45686_txData[0] = ACCEL_DATA_X1_UI | 0x80;  // set MSB = 1 for read
			ICM45686_txData[1] = ICM45686_txData[2] = ICM45686_txData[3] = ICM45686_txData[4] = ICM45686_txData[5] = ICM45686_txData[6] = 0x00;
			ICM45686_txData[7] = ICM45686_txData[8] = ICM45686_txData[9] = ICM45686_txData[10] = ICM45686_txData[11] = ICM45686_txData[12] = 0x00;
			ICM45686_CS_LOW();
			//HAL_SPI_TransmitReceive(&hspi1, ICM45686_txData, ICM45686_rxData, 7, HAL_MAX_DELAY);
			HAL_SPI_TransmitReceive_DMA(&hspi1, ICM45686_txData, ICM45686_rxData, 13);
		}
	}

	if(BMI160_INT_FLAG == 0)
	{
		if (GPIO_Pin == GPIO_PIN_15)
		{
			BMI160_txData[0] = GYR_X_L | 0x80;  // set MSB = 1 for read
			BMI160_txData[1] = BMI160_txData[2] = BMI160_txData[3] = BMI160_txData[4] = BMI160_txData[5] = BMI160_txData[6] = BMI160_txData[7] = BMI160_txData[8] = BMI160_txData[9] = BMI160_txData[10] = BMI160_txData[11] = BMI160_txData[12] = 0x00; // dummy
			BMI160_CS_LOW();
			HAL_SPI_TransmitReceive_DMA(&hspi2, BMI160_txData, BMI160_rxData, 13);
		}
	}
}
/* USER CODE END 1 */

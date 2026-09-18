#include "fs-ia6b.h"

FSiA6B_iBus iBus;

int rh,rv,lv,lh,swa, swc;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len)
{
	unsigned short chksum = 0xffff;

	for(int i=0; i<len-2; i++)
	{
		chksum = chksum - data[i];
	}


	return ((chksum&0x00ff)==data[30]  &&  (chksum>>8)==data[31]);
}



void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus)
{
	iBus->RH = (data[2] | data[3]<<8) & 0x0fff;
	rh = iBus->RH;
	iBus->RV = (data[4] | data[5]<<8) & 0x0fff;
	rv = iBus->RV;
	iBus->LV = (data[6] | data[7]<<8) & 0x0fff;
	lv = iBus->LV;
	iBus->LH = (data[8] | data[9]<<8) & 0x0fff;
	lh = iBus->LH;
	iBus->SwA = (data[10] | data[11]<<8) & 0x0fff;
	swa = iBus->SwA;
	iBus->SwC = (data[12] | data[13]<<8) & 0x0fff;
	swc = iBus->SwC;

	iBus->FailSafe = (data[13] >> 4);
}


unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus)
{
	return iBus->FailSafe != 0;
}

//void FSiA6B_UART5_Initialization(void)
//{
//	/* USER CODE BEGIN UART5_Init 0 */
//
//	  /* USER CODE END UART5_Init 0 */
//
//	  LL_USART_InitTypeDef UART_InitStruct = {0};
//
//	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//	  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//
//	  /** Initializes the peripherals clock
//	  */
//	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
//	  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
//	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }
//
//	  /* Peripheral clock enable */
//	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
//
//	  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
//	  /**UART5 GPIO Configuration
//	  PB12   ------> UART5_RX
//	  PB13   ------> UART5_TX
//	  */
//	  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13;
//	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//	  GPIO_InitStruct.Alternate = LL_GPIO_AF_14;
//	  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	  /* UART5 interrupt Init */
//	  NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
//	  NVIC_EnableIRQ(UART5_IRQn);
//
//	  /* USER CODE BEGIN UART5_Init 1 */
//
//	  /* USER CODE END UART5_Init 1 */
//	  UART_InitStruct.BaudRate = 115200;
//	  UART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
//	  UART_InitStruct.StopBits = LL_USART_STOPBITS_1;
//	  UART_InitStruct.Parity = LL_USART_PARITY_NONE;
//	  UART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
//	  UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
//	  UART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
//	  UART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
//
//	  LL_USART_Init(UART5, &UART_InitStruct);
//	  LL_USART_DisableFIFO(UART5);
//	  LL_USART_SetTXFIFOThreshold(UART5, LL_USART_FIFOTHRESHOLD_1_8);
//	  LL_USART_SetRXFIFOThreshold(UART5, LL_USART_FIFOTHRESHOLD_1_8);
//	  LL_USART_ConfigAsyncMode(UART5);
//
//	  /* USER CODE BEGIN WKUPType UART5 */
//
//	  /* USER CODE END WKUPType UART5 */
//
//	  LL_USART_Enable(UART5);
//
//	  /* Polling UART5 initialisation */
//	  while((!(LL_USART_IsActiveFlag_TEACK(UART5))) || (!(LL_USART_IsActiveFlag_REACK(UART5))))
//	  {
//	  }
//	  /* USER CODE BEGIN UART5_Init 2 */
//
//	  /* USER CODE END UART5_Init 2 */
//}

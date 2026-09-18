
#include <m8n.h>
#include "main.h"

M8N_UBX_NAV_PVT pvt;
//extern int counter;
float M8NHeight, M8NLattitude, M8NLongtitude, M8NVELN, M8NVELE, M8NVELD;

const unsigned char UBX_CFG_PRT[] = {
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
	0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x01, 0x00,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x79
};

//const unsigned char UBX_CFG_MSG[] = {
//	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01,
//	0x00, 0x00, 0x00, 0x00, 0x13, 0xBE
//};
const unsigned char UBX_CFG_MSG[] = {
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x18, 0xE1
};

const unsigned char UBX_CFG_RATE[] = {
	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00,
	0x01, 0x00, 0xDE, 0x6A
};

const unsigned char UBX_CFG_CFG[] = {
	0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31,
	0xBF
};


//this function to transmit the config data sets stored in these arrays to M8N via UART4
void M8N_TransmitData(unsigned char* data, unsigned char len)
{
	for(int i = 0; i < len; i++)
	{

		//counter++;
		while(!LL_USART_IsActiveFlag_TXE(UART4));
		//counter++;
		LL_USART_TransmitData8(UART4, *(data+i));

	}
}

//void M8N_UART4_Initialization(void)
//{
//	LL_USART_InitTypeDef UART_InitStruct = {0};
//
//	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//	  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//
//	  /** Initializes the peripherals clock
//	  */
//	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
//	  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
//	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }
//
//	  /* Peripheral clock enable */
//	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
//
//	  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
//	  /**UART4 GPIO Configuration
//	  PA0   ------> UART4_TX
//	  PA1   ------> UART4_RX
//	  */
//	  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
//	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//	  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
//	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//	  /* UART4 interrupt Init */
//	  NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
//	  NVIC_EnableIRQ(UART4_IRQn);
//
//	  /* USER CODE BEGIN UART4_Init 1 */
//
//	  /* USER CODE END UART4_Init 1 */
//	  UART_InitStruct.BaudRate = 9600;
//	  UART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
//	  UART_InitStruct.StopBits = LL_USART_STOPBITS_1;
//	  UART_InitStruct.Parity = LL_USART_PARITY_NONE;
//	  UART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
//	  UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
//	  UART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
//	  UART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
//
//	  LL_USART_Init(UART4, &UART_InitStruct);
//	  LL_USART_DisableFIFO(UART4);
//	  LL_USART_SetTXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
//	  LL_USART_SetRXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
//	  LL_USART_ConfigAsyncMode(UART4);
//
//	  /* USER CODE BEGIN WKUPType UART4 */
//
//	  /* USER CODE END WKUPType UART4 */
//
//	  LL_USART_Enable(UART4);
//
//	  /* Polling UART4 initialisation */
//	  while((!(LL_USART_IsActiveFlag_TEACK(UART4))) || (!(LL_USART_IsActiveFlag_REACK(UART4))))
//	  {
//	  }
//	  /* USER CODE BEGIN UART4_Init 2 */
//
//	  /* USER CODE END UART4_Init 2 */
//}

void M8N_Initialization(void)
{

	//M8N_UART4_Initialization();
	//counter++;
	//M8N_TransmitData(&UBX_CFG_PRT[0], sizeof(UBX_CFG_PRT));
	//counter++;
	//HAL_Delay(100);
//	M8N_TransmitData(&UBX_CFG_MSG[0], sizeof(UBX_CFG_MSG));
//	HAL_Delay(100);
//	M8N_TransmitData(&UBX_CFG_RATE[0], sizeof(UBX_CFG_RATE));
//	HAL_Delay(100);
//	M8N_TransmitData(&UBX_CFG_CFG[0], sizeof(UBX_CFG_CFG));

}



int class, id;

unsigned char M8N_UBX_CHKSUM_Check(unsigned char* data, unsigned char len)
{
	unsigned char CK_A = 0, CK_B = 0;

	for(int i=2; i<len-2; i++)
	{
		CK_A = CK_A + data[i];
		CK_B = CK_B + CK_A;
	}

	return((CK_A == data[len-2])  &&  (CK_B == data[len-1]));
}



//int AcknowledgedClassID, AcknowledgedMassageID;
//void M8N_UBX_ACK_ACK_Parsing(unsigned char* data, M8N_UBX_NAV_POSLLH* posllh)
//{
//	posllh->CLASS = data[2];
//	posllh-> ID = data[3];
//	posllh->length = data[4] | data[5]<<8;
//
//	posllh-> clsID = data[6];
//	AcknowledgedClassID = posllh-> clsID;
//	posllh-> msgID = data[7];
//	AcknowledgedMassageID = posllh-> msgID;
//}
//void M8N_UBX_ACK_NACK_Parsing(unsigned char* data, M8N_UBX_NAV_POSLLH* posllh)
//{
//	posllh->CLASS = data[2];
//	posllh-> ID = data[3];
//	posllh->length = data[4] | data[5]<<8;
//
//	posllh-> clsID = data[6];
//	AcknowledgedClassID = posllh-> clsID;
//	posllh-> msgID = data[7];
//	AcknowledgedMassageID = posllh-> msgID;
//}

void M8N_UBX_NAV_PVT_Parsing(unsigned char* data, M8N_UBX_NAV_PVT* pvt)
{
	pvt->CLASS	= 	data[2];
	pvt-> ID 	= 	data[3];
	pvt->length = 	data[4]   | data[5]<<8;
	pvt->iTOW 	= 	data[6]   | data[7]<<8  | data[8]<<16  | data[9]<<24;
	pvt->lon 	= 	data[30]  | data[31]<<8 | data[32]<<16 | data[33]<<24;
	pvt->lat 	= 	data[34]  | data[35]<<8 | data[36]<<16 | data[37]<<24;
	pvt->height = 	data[38]  | data[39]<<8 | data[40]<<16 | data[41]<<24;
	pvt->hMSL 	= 	data[42]  | data[43]<<8 | data[44]<<16 | data[45]<<24;
	pvt->hACC 	= 	data[46]  | data[47]<<8 | data[48]<<16 | data[49]<<24;
	pvt->vACC 	= 	data[50]  | data[51]<<8 | data[52]<<16 | data[53]<<24;
	pvt->velN 	= 	data[54]  | data[55]<<8 | data[56]<<16 | data[57]<<24;
	pvt->velE 	= 	data[58]  | data[59]<<8 | data[60]<<16 | data[61]<<24;
	pvt->velD 	= 	data[62]  | data[63]<<8 | data[64]<<16 | data[65]<<24;

	//pvt->lon_f64 = pvt->lon / 10000000.;
	//pvt->lat_f64 = pvt->lat / 10000000.;


	M8NLongtitude 	=   (pvt->lon) 		*0.0000001;
	M8NLattitude 	= 	(pvt->lat)		*0.0000001;
	id 				= 	(pvt->ID);
	class 			= 	(pvt->CLASS);
	M8NHeight 		= 	(pvt->height)	*0.001;
	M8NVELN 		= 	(pvt->velN)		*0.001;
	M8NVELE 		= 	(pvt->velE)		*0.001;
	M8NVELD 		= 	(pvt->velD)		*0.001;


}

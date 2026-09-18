#include <dps310.h>
#include <main.h>
#include <stdio.h>
#include <math.h>
#include <stm32h7xx_hal.h>

extern SPI_HandleTypeDef hspi3;
#define DELAY 100



int16_t c0, c1, c01, c11, c20, c21, c30;
int32_t c00, c10,a,b,p6,p7;
float c,dps_temp,p8,p9, height2, height1;
uint8_t data[3];
uint8_t cfg, x,p1,p2,p3,p4,p5,k;

int y;

//uint32_t start_dps, stop_dps;
//float us_dps;

void DPS310_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t txData[2];
    txData[0] = reg & 0x7F;  // MSB=0 for write
    txData[1] = value;
    DPS310_CS_LOW();
    HAL_SPI_Transmit(&hspi3, txData, 2, DELAY);
    DPS310_CS_HIGH();
}

// ------------ READ FUNCTION (separated TX + RX) ------------
uint8_t DPS310_ReadReg(uint8_t reg)
{
	uint8_t txData1[2];
	uint8_t rxData1[2];
	txData1[0] = reg | 0x80;  // set MSB = 1 for read
	txData1[1] = 0x00;        // dummy
	DPS310_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi3, txData1, rxData1, 2, DELAY);
	DPS310_CS_HIGH();

	return rxData1[1];
}


	// Valid up to 31 bits
	static inline int32_t twos_complement (const uint32_t value, const uint8_t bits)
	{
		const int32_t max_signed_value_exclusive = (int32_t) (1U << (bits - 1U));
		int32_t complement = (int32_t) value;

		if (complement >= max_signed_value_exclusive)
		{
			complement -= 2 * max_signed_value_exclusive;
		}

		return complement;
	}


void DPS310_Initialization(void)
{
    uint8_t whoami = DPS310_ReadReg(DPS310_CHIPID);
    HAL_Delay(10);
//    if (whoami == DPS310_CHIPID_VALUE)   // expected value from datasheet
//    {
//    	//printf("GOOOOOOOOOOOOOOOOd\n");
//    }
//    else
//    {
//    	//printf("FUUUUUUUCK\n");
//    }
//    //printf("%d\n", whoami);
    //k = whoami;



    //READING CALIBRATION COEFFITIENTS
    uint8_t COEF[17] = {0};
	COEF[0] = DPS310_ReadReg(CAL_COEF_0);
	COEF[1] = DPS310_ReadReg(CAL_COEF_1);
	COEF[2] = DPS310_ReadReg(CAL_COEF_2);
	int16_t raw_c0 = (((uint16_t)COEF[0] << 4) | ((COEF[1] >> 4) & 0b00001111));//////////
	c0 = (int16_t)twos_complement (raw_c0, 12);
	int16_t raw_c1 = (((uint16_t)(COEF[1] & 0b00001111) << 8) | COEF[2]);
	c1 = twos_complement (raw_c1, 12);

	COEF[3] = DPS310_ReadReg(CAL_COEF_3);
	COEF[4] = DPS310_ReadReg(CAL_COEF_4);
	COEF[5] = DPS310_ReadReg(CAL_COEF_5);
	COEF[6] = DPS310_ReadReg(CAL_COEF_6);
	COEF[7] = DPS310_ReadReg(CAL_COEF_7);
	int32_t raw_c00 = (((uint32_t)COEF[3] << 12) | ((uint32_t)COEF[4] << 4) | ((COEF[5] >> 4) & 0b00001111));////////////
	c00 = twos_complement (raw_c00, 20);
	int32_t raw_c10 = ((uint32_t)(COEF[5] & 0b00001111) << 16 | ((uint32_t)COEF[6] << 8) | COEF[7]);
	c10 = twos_complement (raw_c10, 20);

	COEF[8] = DPS310_ReadReg(CAL_COEF_8);
	COEF[9] = DPS310_ReadReg(CAL_COEF_9);
	int16_t raw_c01 = ((uint32_t)COEF[8] << 8 | COEF[9]);
	c01 = twos_complement (raw_c01, 16);

	COEF[10] = DPS310_ReadReg(CAL_COEF_10);
	COEF[11] = DPS310_ReadReg(CAL_COEF_11);
	int16_t raw_c11 = ((uint32_t)COEF[10] << 8 | COEF[11]);
	c11 = twos_complement (raw_c11, 16);

	COEF[12] = DPS310_ReadReg(CAL_COEF_12);
	COEF[13] = DPS310_ReadReg(CAL_COEF_13);
	int16_t raw_c20 = ((uint32_t)COEF[12] << 8 | COEF[13]);
	c20 = twos_complement (raw_c20, 16);

	COEF[14] = DPS310_ReadReg(CAL_COEF_14);
	COEF[15] = DPS310_ReadReg(CAL_COEF_15);
	int16_t raw_c21 = ((uint32_t)COEF[14] << 8 | COEF[15]);
	c21 = twos_complement (raw_c21, 16);

	COEF[16] = DPS310_ReadReg(CAL_COEF_16);
	COEF[17] = DPS310_ReadReg(CAL_COEF_17);
	int16_t raw_c30 = ((uint32_t)COEF[16] << 8 | COEF[17]);
	c30 = twos_complement (raw_c30, 16);


	DPS310_WriteReg(PRS_CFG, PRS_CFG_VALUE);		//pressure measurement rate = 4 measurment per second
	uint8_t x = DPS310_ReadReg(PRS_CFG);			//pressure oversampling rate = 8 times(when changing value to >8 be careful to register 0x09)
	HAL_Delay(10);

	x = DPS310_ReadReg(TMP_COEF_SRCE);				//States which internal temperature sensor the calibration coefficients are based on
	HAL_Delay(10);									//its decimal value is 181---> 10110101 in binary

	DPS310_WriteReg(TMP_CFG, TMP_CFG_VALUE);		//tempreture measurement rate = 4 measurment per second
	x = DPS310_ReadReg(TMP_CFG);					//tempreture oversampling rate = 8 times(when changing value to >8 be careful to register 0x09)
	HAL_Delay(10);


	DPS310_WriteReg(CFG_REG, CFG_REG_VALUE);		//enabaling inttrupts on SD0 and it is active HIHG
	x = DPS310_ReadReg(CFG_REG);					//
	HAL_Delay(10);

//	DPS310_WriteReg(MEAS_CFG, 0x02);				//temprature measurment READY
//	x = DPS310_ReadReg(MEAS_CFG);					//
//	HAL_Delay(10);
//
//	DPS310_WriteReg(MEAS_CFG, 0x01);				//pressure measurment READY
//	x = DPS310_ReadReg(MEAS_CFG);					//
//	HAL_Delay(10);

		cfg = DPS310_ReadReg(MEAS_CFG);
		p1 = cfg;
		cfg = (cfg & 0xF8) | 0x07;   // Clear only bits [2:0], set MEAS_CTRL = 001
		DPS310_WriteReg(MEAS_CFG, cfg);				//Pressure measurment READY

}





void DPS310_GetTempPres(Struct_DPS310 *DataStruct)
{


	x = DPS310_ReadReg(MEAS_CFG);
	if((x & (1 << 5)) == 0  &&  (x & (1 << 4)) == 0)
		{
			//y++;
			return;
		}

	//cfg = DPS310_ReadReg(MEAS_CFG);
	//cfg = (cfg & 0xF8) | 0x07;   				// Clear only bits [2:0], set MEAS_CTRL = 010
	//DPS310_WriteReg(MEAS_CFG, cfg);				//temprature measurment READY

	//x = DPS310_ReadReg(MEAS_CFG);
//	//HAL_Delay(40);
//x = DPS310_ReadReg(MEAS_CFG);
////	while ((x & (1 << 5)) == 0)  // Wait for TMP_RDY
////	{
//		//x++;
// x = DPS310_ReadReg(MEAS_CFG);
////	}
//
//	//start_dps = DWT->CYCCNT;
//	data[0] = DPS310_ReadReg(TMP_B0);
//	//stop_dps = DWT->CYCCNT;
//	//us_dps = (stop_dps - start_dps) / (SystemCoreClock / 1000000.0f); // convert to microseconds
//	data[1] = DPS310_ReadReg(TMP_B1);
//	data[2] = DPS310_ReadReg(TMP_B2);

//y--;

	uint8_t txData[7];
	uint8_t rxData[7];

	txData[0] = PSR_B2 | 0x80;  // set MSB = 1 for read
	txData[1] = txData[2] = txData[3] = txData[4] = txData[5] = txData[6] = 0x00;        // dummy



	DPS310_CS_LOW();
	//start_dps = DWT->CYCCNT;
	HAL_SPI_TransmitReceive(&hspi3, txData, rxData, 7, DELAY);
	//stop_dps = DWT->CYCCNT;
	//us_dps = (stop_dps - start_dps) / (SystemCoreClock / 1000000.0f); // convert to microseconds
	DPS310_CS_HIGH();






	DataStruct->Temp_Raw = 0;
	DataStruct->Temp_Raw = (uint32_t)((rxData[4] << 16) | (rxData[5] << 8) | rxData[6]);

	DataStruct->Temp_Value = twos_complement(DataStruct->Temp_Raw, 24);

	DataStruct->Temp_Value_Scaled = (float)DataStruct->Temp_Value / 7864320.0f;

	DataStruct->Temp_Compensated = (float)c0*0.5f + (float)c1*(DataStruct->Temp_Value_Scaled);
	dps_temp = DataStruct->Temp_Compensated;





	//cfg = DPS310_ReadReg(MEAS_CFG);
	//p1 = cfg;
	//cfg = (cfg & 0xF8) | 0x07;   // Clear only bits [2:0], set MEAS_CTRL = 001
	//DPS310_WriteReg(MEAS_CFG, cfg);				//Pressure measurment READY
	//HAL_Delay(40);
	//y = DPS310_ReadReg(MEAS_CFG);
	//p2 = x;
	//while ((y & (1 << 4)) == 0)  // Wait for PSR_RDY
		//{
		    //y = DPS310_ReadReg(MEAS_CFG);
		//}

	//data[0] = DPS310_ReadReg(PSR_B0);
	//data[1] = DPS310_ReadReg(PSR_B1);
	//data[2] = DPS310_ReadReg(PSR_B2);

	DataStruct->Pres_Raw = (uint32_t)((rxData[1] << 16) | (rxData[2] << 8) | rxData[3]);
	//start_dps = DWT->CYCCNT;
	DataStruct->Pres_Value = twos_complement (DataStruct->Pres_Raw, 24);
	//stop_dps = DWT->CYCCNT;
	//us_dps = (stop_dps - start_dps) / (SystemCoreClock / 1000000.0f); // convert to microseconds
	DataStruct->Pres_Value_Scaled = (float)DataStruct->Pres_Value/7864320.0f;

	//start_dps = DWT->CYCCNT;
	DataStruct->Pres_Compensated = (float)c00 +
			(float)(DataStruct->Pres_Value_Scaled) * ((float)c10+DataStruct->Pres_Value_Scaled*((float)c20+(float)(DataStruct->Pres_Value_Scaled)*(float)c30)) +
			(float)(DataStruct->Temp_Value_Scaled) * (float)c01 +
			(float)(DataStruct->Temp_Value_Scaled)*((float)DataStruct->Pres_Value_Scaled)*((float)c11+(float)(DataStruct->Pres_Value_Scaled)*(float)c21);
	//stop_dps = DWT->CYCCNT;
	//us_dps = (stop_dps - start_dps) / (SystemCoreClock / 1000000.0f); // convert to microseconds

	//float PoP = DataStruct->Pres_Compensated / 101325.0;
	//float PpN = pow(PoP, 0.190284);  //PoP^0.190284;
	height1 = (float)44330 * (1.0f - pow(DataStruct->Pres_Compensated / 101325.0, 0.190284));



}

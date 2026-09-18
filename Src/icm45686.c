//most of the variables are global(defined outside their function),
//so we can use directly these variables for debugging and to see final results



#include <icm45686.h>
#include <main.h>
#include <stdio.h>
#include <math.h>
#include "stm32h7xx_hal.h"
#include "notchfilter.h"
#include "lpf2p.h"


#define RAD_TO_DEG 57.295779513082320876798154814105
const float icm45686_Accel_Z_corrector = 2048.0;

extern SPI_HandleTypeDef hspi1;
extern Struct_ICM45686 ICM45686;
extern Axis3f icm45686_gyro;
extern Axis3f icm45686_acc;
extern Axis3f_Bias gyro_bias;
extern lpf2pData lpfData_GX, lpfData_GY, lpfData_GZ;

//extern NotchFilter notchFilt;
extern NotchFilterFloat First_Harmonic_Notch_GX,  First_Harmonic_Notch_GY, First_Harmonic_Notch_GZ;
extern NotchFilterFloat Second_Harmonic_Notch_GX, Second_Harmonic_Notch_GY, Second_Harmonic_Notch_GZ;
extern NotchFilterFloat Third_Harmonic_Notch_GX,  Third_Harmonic_Notch_GY, Third_Harmonic_Notch_GZ;

float rollfromacc, pitchfromacc;

//int xx;
uint32_t start_icm, stop_icm;
float us_icm, usApplyNotchArdu, usApplyNotchPhils;

float icm_roll, icm_roll_bias, icm_roll_used_for_calibration;
float icm_pitch, icm_pitch_bias, icm_pitch_used_for_calibration;

volatile float ICM_GX, ICM_GX_100Fc_LPF, ICM_AX, ICM_AY, ICM_AZ, ICM_GX_First_Harmonic_Notch, ICM_GX_Second_Harmonic_Notch;
volatile float ICM_GY, ICM_GY_100Fc_LPF,ICM_GY_First_Harmonic_Notch, ICM_GY_Second_Harmonic_Notch;
volatile float ICM_GZ, ICM_GZ_100Fc_LPF,ICM_GZ_First_Harmonic_Notch, ICM_GZ_Second_Harmonic_Notch;
volatile float ICM_GX_UnBiased, ICM_GY_UnBiased, ICM_GZ_UnBiased;

extern uint8_t tim2_10ms_flag;





icm45686_Kalman_t icm45686_KalmanX = {
    .Q_angle = 0.001f,// (interpreted as deg^2 / s)
    .Q_bias = 0.003f,// (interpreted as deg^2 / s^3)
    .R_measure = 0.03f};// deg^2

icm45686_Kalman_t icm45686_KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.001f,
    .R_measure = 0.03f,
};






//LL driver function
//unsigned char SPI1_SendByte(unsigned char data)
//{
//
//	while(LL_SPI_IsActiveFlag_TXP(ICM45686_SPI_CHANNEL)==RESET);
//	LL_SPI_TransmitData8(ICM45686_SPI_CHANNEL, data);
//
//	while(LL_SPI_IsActiveFlag_RXP(ICM45686_SPI_CHANNEL)==RESET);
//	xx++;
//	return LL_SPI_ReceiveData8(ICM45686_SPI_CHANNEL);
//}
//
////////////////////////////////////////////////////////////////
//
//uint8_t ICM45686_Readbyte(uint8_t reg_addr)
//{
//	uint8_t val;
//
//	ICM45686_CS_LOW();
//	//should enter a very small delay
//	SPI1_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
//	val = SPI1_SendByte(0x00); //Send DUMMY to read data
//	ICM45686_CS_HIGH();
//
//	return val;
//}
//
//void ICM45686_Writebyte(uint8_t reg_addr, uint8_t val)
//{
//	ICM45686_CS_LOW();
//	//should enter a delay
//	SPI1_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.
//	SPI1_SendByte(val); //Send Data to write
//	ICM45686_CS_HIGH();
//}
//
















// Single register read
uint8_t ICM45686_ReadReg(uint8_t reg)
{
    uint8_t txData[2];
    uint8_t rxData[2];

    txData[0] = reg | 0x80;  // set MSB = 1 for read
    txData[1] = 0x00;        // dummy

    ICM45686_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, HAL_MAX_DELAY);
    ICM45686_CS_HIGH();

    return rxData[1];   // second byte is the register value
}

void ICM45686_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t txData[2];
    txData[0] = reg & 0x7F;  // MSB=0 for write
    txData[1] = value;
    ICM45686_CS_LOW();
    HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
    ICM45686_CS_HIGH();
}

void ICM45686_Initialization()
{

    uint8_t whoami = ICM45686_ReadReg(0X72);
    HAL_Delay(20);

    ICM45686_WriteReg(PWR_MGMT0, PWR_MGMT0_VALUE);
    HAL_Delay(20);



   ICM45686_WriteReg(ACCEL_CONFIG0, ACCEL_CONFIG0_VALUE);		//acc_range = +-2g and acc_ord = 1600hz
   HAL_Delay(20);



//  ICM45686_WriteReg(GYRO_CONFIG0, GYRO_CONFIG0_VALUE);		//gyro_range = +-15deg and gyr_odr = 1600hz
//  HAL_Delay(20);
//  x = ICM45686_ReadReg(GYRO_CONFIG0);

   ICM45686_WriteReg(GYRO_CONFIG0, GYRO_CONFIG0_VALUE);		//gyro_range = +-250deg and gyr_odr = 1600hz
   HAL_Delay(20);


   ICM45686_WriteReg(INT1_CONFIG0, INT1_CONFIG0_VALUE);		//gyr_odr = 1600hz
   HAL_Delay(20);


   ICM45686_WriteReg(INT1_CONFIG2, INT1_CONFIG2_VALUE);
   HAL_Delay(20);



    for(int i = 0; i<2000; i++)
    {
    	ICM45686_Get3AxisGyroRawData(&ICM45686, &icm45686_gyro, &icm45686_acc, &gyro_bias);
    	gyro_bias.x += (float)icm45686_gyro.x;
    	gyro_bias.y += (float)icm45686_gyro.y;
    	gyro_bias.z += (float)icm45686_gyro.z;

    	icm_pitch_bias += (float)icm_pitch_used_for_calibration;
    	icm_roll_bias  += (float)icm_roll_used_for_calibration;

    }
    	gyro_bias.x = (float)gyro_bias.x / 2000.0;
    	gyro_bias.y = (float)gyro_bias.y / 2000.0;
    	gyro_bias.z = (float)gyro_bias.z / 2000.0;

    	icm_pitch_bias = (float)icm_pitch_bias / 2000.0;
    	icm_roll_bias  = (float)icm_roll_bias  / 2000.0;



}



//void ICM45686_Get3AxisAccRawData(Struct_ICM45686 *DataStruct)
//{
//
//		uint8_t data[6];
//
//	    data[0] = ICM45686_ReadReg(ACCEL_DATA_X0_UI);
//	    //aa0 = data[0];
//	    data[1] = ICM45686_ReadReg(ACCEL_DATA_X1_UI);
//	    //aa1 = data[1];
//	    data[2] = ICM45686_ReadReg(ACCEL_DATA_Y0_UI);
//	    //aa2 = data[2];
//	    data[3] = ICM45686_ReadReg(ACCEL_DATA_Y1_UI);
//	   // aa3 = data[3];
//	    data[4] = ICM45686_ReadReg(ACCEL_DATA_Z0_UI);
//	    //aa4 = data[4];
//	    data[5] = ICM45686_ReadReg(ACCEL_DATA_Z1_UI);
//	    //aa5 = data[5];
//
//	    DataStruct->Accel_X_RAW = (int16_t)((data[0] << 8) | data[1]);
//	    DataStruct->Accel_Y_RAW = (int16_t)((data[2] << 8) | data[3]);
//	    DataStruct->Accel_Z_RAW = (int16_t)((data[4] << 8) | data[5]);
//
//	    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
//	    AX = DataStruct->Ax;
//	    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
//	    AZ = DataStruct->Ay;
//	    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
//	    AY = DataStruct->Az;
//
//
//}





//start_icm = DWT->CYCCNT;
//stop_icm = DWT->CYCCNT;
//us_icm = (stop_icm - start_icm) / (SystemCoreClock / 1000000.0f); // convert to microseconds
extern uint8_t ICM45686_txData[13];
extern uint8_t ICM45686_rxData[13];
void ICM45686_Get3AxisGyroRawData(Struct_ICM45686 *DataStruct, Axis3f *icm45686_gyro, Axis3f *icm45686_acc, Axis3f_Bias *gyro_bias)
{


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




//	ICM45686_txData[0] = GYRO_DATA_X1_UI | 0x80;  // set MSB = 1 for read
//	ICM45686_txData[1] = 0x00;        // dummy
//	ICM45686_txData[2] = 0x00;
//	ICM45686_txData[3] = 0x00;
//	ICM45686_txData[4] = 0x00;
//	ICM45686_txData[5] = 0x00;
//	ICM45686_txData[6] = 0x00;

//	ICM45686_CS_LOW();
//	HAL_SPI_TransmitReceive(&hspi1, ICM45686_txData, ICM45686_rxData, 7, HAL_MAX_DELAY);
//	ICM45686_CS_HIGH();





    DataStruct->Gyro_X_RAW = (int16_t)((ICM45686_rxData[8] << 8)  | ICM45686_rxData[7]);
    DataStruct->Gyro_Y_RAW = (int16_t)((ICM45686_rxData[10] << 8) | ICM45686_rxData[9]);
    DataStruct->Gyro_Z_RAW = (int16_t)((ICM45686_rxData[12] << 8) | ICM45686_rxData[11]);
    DataStruct->Gx   =  DataStruct->Gyro_X_RAW / 131.0;
    icm45686_gyro->x = -DataStruct->Gx;
    ICM_GX_UnBiased  = -DataStruct->Gx - gyro_bias->x;


    DataStruct->Gy   = DataStruct->Gyro_Y_RAW / 131.0;
    icm45686_gyro->y = DataStruct->Gy;
    ICM_GY_UnBiased  = DataStruct->Gy - gyro_bias->y;

    DataStruct->Gz   =  DataStruct->Gyro_Z_RAW / 131.0;
    icm45686_gyro->z =  DataStruct->Gz;
    ICM_GZ_UnBiased  = -(DataStruct->Gz - gyro_bias->z);




//////////////////////////////////////////////////
/////////////////GYRO_X_FILTERING////////////////
////////////////////////////////////////////////
ICM_GX_100Fc_LPF = lpf2pApply(&lpfData_GX, ICM_GX_UnBiased);

//start_icm = DWT->CYCCNT;
ICM_GX_First_Harmonic_Notch = notch_apply(&First_Harmonic_Notch_GX, ICM_GX_100Fc_LPF);
//stop_icm = DWT->CYCCNT;
//usApplyNotchArdu = (stop_icm - start_icm) / (SystemCoreClock / 1000000.0f); // convert to microseconds

ICM_GX_Second_Harmonic_Notch = notch_apply(&Second_Harmonic_Notch_GX, ICM_GX_First_Harmonic_Notch);
ICM_GX 						=  notch_apply(&Third_Harmonic_Notch_GX, ICM_GX_Second_Harmonic_Notch);

//////////////////////////////////////////////////
/////////////////GYRO_Y_FILTERING////////////////
////////////////////////////////////////////////
ICM_GY_100Fc_LPF = lpf2pApply(&lpfData_GY, ICM_GY_UnBiased);

//start_icm = DWT->CYCCNT;
ICM_GY_First_Harmonic_Notch = notch_apply(&First_Harmonic_Notch_GY, ICM_GY_100Fc_LPF);
//stop_icm = DWT->CYCCNT;
//usApplyNotchArdu = (stop_icm - start_icm) / (SystemCoreClock / 1000000.0f); // convert to microseconds

ICM_GY_Second_Harmonic_Notch = notch_apply(&Second_Harmonic_Notch_GY, ICM_GY_First_Harmonic_Notch);
ICM_GY 						=  notch_apply(&Third_Harmonic_Notch_GY, ICM_GY_Second_Harmonic_Notch);


//////////////////////////////////////////////////
/////////////////GYRO_Z_FILTERING////////////////
////////////////////////////////////////////////
ICM_GZ_100Fc_LPF = lpf2pApply(&lpfData_GZ, ICM_GZ_UnBiased);

//start_icm = DWT->CYCCNT;
ICM_GZ_First_Harmonic_Notch = notch_apply(&First_Harmonic_Notch_GZ, ICM_GZ_100Fc_LPF);
//stop_icm = DWT->CYCCNT;
//usApplyNotchArdu = (stop_icm - start_icm) / (SystemCoreClock / 1000000.0f); // convert to microseconds

ICM_GZ_Second_Harmonic_Notch = notch_apply(&Second_Harmonic_Notch_GZ, ICM_GZ_First_Harmonic_Notch);
ICM_GZ 						 = notch_apply(&Third_Harmonic_Notch_GZ,  ICM_GZ_Second_Harmonic_Notch);








    DataStruct->Accel_X_RAW = (int16_t)((ICM45686_rxData[2] << 8) | ICM45686_rxData[1]);
	DataStruct->Accel_Y_RAW = (int16_t)((ICM45686_rxData[4] << 8) | ICM45686_rxData[3]);
	DataStruct->Accel_Z_RAW = (int16_t)((ICM45686_rxData[6] << 8) | ICM45686_rxData[5]);
	DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
	icm45686_acc->x = DataStruct->Ax;
	//ICM_AX = DataStruct->Ax;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
	icm45686_acc->y = DataStruct->Ay;
	//ICM_AY = DataStruct->Ay;
	DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
	icm45686_acc->z = DataStruct->Az;
	//ICM_AZ = DataStruct->Az;


//if(tim2_10ms_flag == 1)
//{

	//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);

	tim2_10ms_flag = 0;
	// Kalman angle solve
		float dt = 0.000625;//(float)(HAL_GetTick() - bmi160_timer) / 1000;
		//bmi160_timer = HAL_GetTick();

							//gyroroll += dt * BMI_GX;
							//gyropitch += dt * BMI_GY;


		float roll;
		//start_icm = DWT->CYCCNT;
		float roll_sqrt = sqrt(DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
		//stop_bmi = DWT->CYCCNT;
			    //us_bmi = (stop_bmi - start_bmi) / (SystemCoreClock / 1000000.0f); // convert to microseconds
		if (roll_sqrt != 0.0)
		{
			start_icm = DWT->CYCCNT;
		        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
		        rollfromacc = roll;
		        //roll = asin(BMI_AX - 0.05)* RAD_TO_DEG;
		        //accroll = roll;

		        //stop_icm = DWT->CYCCNT;
		        //us_icm = (stop_icm - start_icm) / (SystemCoreClock / 1000000.0f); // convert to microseconds

		}
		else
		{
		        roll = 0.0;
		}

		//start_bmi = DWT->CYCCNT;
		float pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
		pitchfromacc = pitch;
		//float pitch = atan2(-BMI_AY+0.04, -BMI_AZ-0.02)* RAD_TO_DEG;
//		stop_icm = DWT->CYCCNT;
//		us_icm = (stop_icm - start_icm) / (SystemCoreClock / 1000000.0f); // convert to microseconds
		//accpitch = pitch;

		if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
		{
		        icm45686_KalmanY.angle = pitch;
		        DataStruct->KalmanAngleY = pitch;
		}
		else
		{
			 //start_bmi = DWT->CYCCNT;
		     DataStruct->KalmanAngleY = ICM45686_Kalman_getAngle1(&icm45686_KalmanY, pitch, icm45686_gyro->y, dt);
		     //stop_bmi = DWT->CYCCNT;
		     //us_bmi = (stop_bmi - start_bmi) / (SystemCoreClock / 1000000.0f); // convert to microseconds

		}

		//start_bmi = DWT->CYCCNT;
		if (fabs(DataStruct->KalmanAngleY) > 90)
		     DataStruct->Gx = -DataStruct->Gx;
		//stop_bmi = DWT->CYCCNT;
		//us_bmi = (stop_bmi - start_bmi) / (SystemCoreClock / 1000000.0f); // convert to microseconds
		DataStruct->KalmanAngleX = ICM45686_Kalman_getAngle1(&icm45686_KalmanX, roll, icm45686_gyro->x, dt);


		//stop_icm = DWT->CYCCNT;
		//us_icm = (stop_icm - start_icm) / (SystemCoreClock / 1000000.0f);

		icm_pitch_used_for_calibration = -(float)DataStruct->KalmanAngleX;
		icm_roll_used_for_calibration = (float)DataStruct->KalmanAngleY;

		icm_pitch = icm_pitch_used_for_calibration - icm_pitch_bias;
		icm_roll = icm_roll_used_for_calibration - icm_roll_bias;


}




float ICM45686_Kalman_getAngle1(icm45686_Kalman_t *Kalman, float newAngle, float newRate, float dt)
{

//		// Enable DWT CYCCNT
//		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//		DWT->CYCCNT = 0;
//		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//
//
//		start_bmi = DWT->CYCCNT;



	//predicted angle
	float rate = newRate - Kalman->bias;//kalman bias is for angular rate
    Kalman->angle += dt * rate;
    //gyropitchinkalmanfilter = Kalman->angle;
    //pridicted covariance matrix
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;
    //kalman gain
    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;
    //correction state vector
    float y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;
    //correction covariance matrix
    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;


    return Kalman->angle;





//    stop_bmi = DWT->CYCCNT;
//    us_bmi = (stop_bmi - start_bmi) / (SystemCoreClock / 1000000.0f); // convert to microseconds
}






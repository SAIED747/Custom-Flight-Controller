#include <bmi160.h>
#include <main.h>
#include <stdio.h>
#include <math.h>
#include <stm32h7xx_hal.h>

extern SPI_HandleTypeDef hspi2;

extern Axis3f bmi160_gyro;
extern Axis3f bmi160_acc;

#include "stm32h7xx_hal.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
const float bmi160_Accel_Z_corrector = 2048.0;



float bmi_roll;
float bmi_pitch;
float BMI_GX, BMI_GY, BMI_GZ, BMI_AX, BMI_AY, BMI_AZ;
float BMI_GX_1,BMI_GY_1,BMI_GZ_1;
float BMI_GX_BIAS, BMI_GY_BIAS, BMI_GZ_BIAS;
float BMI_FREQ, dt, bmi160_timer;
//float Y_BMI_GX_NEW, Y_BMI_GX_PREV;
//float gyroroll, gyropitch, accroll, accpitch;



float dt1;
//uint32_t start_bmi, stop_bmi;
//float us_bmi;



bmi160_Kalman_t bmi160_KalmanX = {
    .Q_angle = 0.001f,// (interpreted as deg^2 / s)
    .Q_bias = 0.003f,// (interpreted as deg^2 / s^3)
    .R_measure = 0.03f};// deg^2

bmi160_Kalman_t bmi160_KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.001f,
    .R_measure = 0.03f,
};

void BMI160_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t txData[2];
    txData[0] = reg & 0x7F;  // MSB=0 for write
    txData[1] = value;
    BMI160_CS_LOW();
    HAL_SPI_Transmit(&hspi2, txData, 2, HAL_MAX_DELAY);
    BMI160_CS_HIGH();
}

// ------------ READ FUNCTION (separated TX + RX) ------------
uint8_t BMI160_ReadReg(uint8_t reg)
{
	uint8_t txData1[2];
	uint8_t rxData1[2];
	txData1[0] = reg | 0x80;  // set MSB = 1 for read
	txData1[1] = 0x00;        // dummy
	BMI160_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi2, txData1, rxData1, 2, HAL_MAX_DELAY);
	BMI160_CS_HIGH();
	return rxData1[1];
}

void BMI160_Initialization1(void)
{
    uint8_t whoami = BMI160_ReadReg(CHIPID);
    HAL_Delay(10);


   BMI160_WriteReg(ACC_CONF, ACC_CONF_VALUE);		//acc_bwp = OSR2---acc_odr = 400hz
   uint8_t x = BMI160_ReadReg(ACC_CONF);
   HAL_Delay(10);

   BMI160_WriteReg(ACC_RANGE, ACC_RANGE_VALUE);		//acc_range = +-2g
   x = BMI160_ReadReg(ACC_RANGE);
   HAL_Delay(10);

   BMI160_WriteReg(GYR_CONF, GYR_CONF_VALUE);		//gyr_bwp = normal---gyr_odr = 400hz
   x = BMI160_ReadReg(GYR_CONF);
   HAL_Delay(10);

   BMI160_WriteReg(GYR_RANGE, GYR_RANGE_VALUE);		//gyr_range = +-125°/s
   x = BMI160_ReadReg(GYR_RANGE);
   HAL_Delay(10);

   BMI160_WriteReg(0x7E, 0x11);  // ACCEL normal mode
   HAL_Delay(5);
   BMI160_WriteReg(CMD, CMD_VALUE);
   HAL_Delay(90);

//   x = BMI160_ReadReg(PMU_STATUS);		//status reading of acc, gyro and magnetometer
//   HAL_Delay(10);

   BMI160_WriteReg(0x51, 0x10);			//enabling data ready
   x = BMI160_ReadReg(0x51);
   HAL_Delay(10);

   BMI160_WriteReg(0x56, 0x80);			//interrupt mapped to pin INT1 when data is ready
   x = BMI160_ReadReg(0x56);
   HAL_Delay(10);

   BMI160_WriteReg(0x53, 0x08);   // INT_OUT_CTRL: INT1 active low, push-pull
   HAL_Delay(10);
   BMI160_WriteReg(0x54, 0x00);
   HAL_Delay(10);


   BMI_GX=0;BMI_GY=0;BMI_GZ=0;
   BMI_GX_1=0;BMI_GY_1=0;BMI_GZ_1=0;
   BMI_GX_BIAS=0;BMI_GY_BIAS=0;BMI_GZ_BIAS=0;
   for(int i = 0; i<200; i++)
   {
	   BMI160_Get6AxisRawData(&BMI160, &bmi160_gyro, &bmi160_acc);
	   BMI_GX_1 += (float)BMI_GX;
	   BMI_GY_1 += (float)BMI_GY;
	   BMI_GZ_1 += (float)BMI_GZ;
   }
   BMI_GX_BIAS = (float)BMI_GX_1/200.0;
   BMI_GY_BIAS = (float)BMI_GY_1/200.0;
   BMI_GZ_BIAS = (float)BMI_GZ_1/200.0;
}


void BMI160_Get3AxisGyroRawData(Struct_BMI160 *DataStruct)
{
    uint8_t data[6];

    data[0] = BMI160_ReadReg(GYR_X_L);
    data[1] = BMI160_ReadReg(GYR_X_H);
    data[2] = BMI160_ReadReg(GYR_Y_L);
    data[3] = BMI160_ReadReg(GYR_Y_H);
    data[4] = BMI160_ReadReg(GYR_Z_L);
    data[5] = BMI160_ReadReg(GYR_Z_H);

    DataStruct->Gyro_X_RAW = (int16_t)((data[1] << 8) | data[0]);
    DataStruct->Gyro_Y_RAW = (int16_t)((data[3] << 8) | data[2]);
    DataStruct->Gyro_Z_RAW = (int16_t)((data[5] << 8) | data[4]);




    DataStruct->Gx = DataStruct->Gyro_X_RAW / 262.4;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 262.4;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 262.4;





//    dt1 = (float)(HAL_GetTick() - bmi160_timer) / 1000;
//    bmi160_timer = HAL_GetTick();








//    BMI_GX = DataStruct->Gx - BMI_GX_BIAS;
//    BMI_GY = DataStruct->Gy - BMI_GY_BIAS;
//    BMI_GZ = DataStruct->Gz - BMI_GZ_BIAS;
//
//
//
//    //LOW-PASS FILTERING
//
//    float dt = 0.000625;  //the ODR = 1600Hz so the sampel period = 0.000625
//
//    BMI_FREQ = 1.0 / dt;	//this the the ODR
//
//    float RC = (1/(2*3.14*100));	//RC is the time constant where 100 in the denominator is the cut-off frequency
//    float alpha = dt/(dt+RC);		//alpha is the smoothing coefficient
//
//
//    //IIR_LPF
//    Y_BMI_GX_NEW = (1-alpha)*Y_BMI_GX_PREV + alpha*BMI_GX;
//    Y_BMI_GX_PREV = Y_BMI_GX_NEW;


}

void BMI160_Get3AxisAccRawData(Struct_BMI160 *DataStruct)
{
	uint8_t data[6];

	    data[0] = BMI160_ReadReg(ACC_X_L);
	    data[1] = BMI160_ReadReg(ACC_X_H);
	    data[2] = BMI160_ReadReg(ACC_Y_L);
	    data[3] = BMI160_ReadReg(ACC_Y_H);
	    data[4] = BMI160_ReadReg(ACC_Z_L);
	    data[5] = BMI160_ReadReg(ACC_Z_H);

	    DataStruct->Accel_X_RAW = (int16_t)((data[1] << 8) | data[0]);
	    DataStruct->Accel_Y_RAW = (int16_t)((data[3] << 8) | data[2]);
	    DataStruct->Accel_Z_RAW = (int16_t)((data[5] << 8) | data[4]);


	    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
	    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
	    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
}


extern uint8_t BMI160_txData[13];
extern uint8_t BMI160_rxData[13];
void BMI160_Get6AxisRawData(Struct_BMI160 *DataStruct, Axis3f *bmi160_gyro, Axis3f *bmi160_acc)
{
//	// Enable DWT CYCCNT
//	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//	DWT->CYCCNT = 0;
//	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;







//	uint8_t txData[13];
//	uint8_t rxData[13];

//	txData[0] = GYR_X_L | 0x80;  // set MSB = 1 for read
//	txData[1] = txData[2] = txData[3] = txData[4] = txData[5] = txData[6] = txData[7] = txData[8] = txData[9] = txData[10] = txData[11] = txData[12] = 0x00; // dummy
//
//
//	BMI160_CS_LOW();
//	//start_bmi = DWT->CYCCNT;
//	HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 13, HAL_MAX_DELAY);
//	//stop_bmi = DWT->CYCCNT;
//	//us_bmi = (stop_bmi - start_bmi) / (SystemCoreClock / 1000000.0f); // convert to microseconds
//	BMI160_CS_HIGH();



	//start_bmi = DWT->CYCCNT;

	DataStruct->Accel_X_RAW = (int16_t)((BMI160_rxData[8] << 8)  | BMI160_rxData[7]);
	DataStruct->Accel_Y_RAW = (int16_t)((BMI160_rxData[10] << 8) | BMI160_rxData[9]);
	DataStruct->Accel_Z_RAW = (int16_t)((BMI160_rxData[12] << 8) | BMI160_rxData[11]);



	DataStruct->Ax = (DataStruct->Accel_X_RAW / 16384.0) +0.012;
	bmi160_acc->x = DataStruct->Ax;

	DataStruct->Ay = (DataStruct->Accel_Y_RAW / 16384.0) +0.01;
	bmi160_acc->y = DataStruct->Ay;

	DataStruct->Az = (DataStruct->Accel_Z_RAW / 16384.0) - 0.022;
	bmi160_acc->z = DataStruct->Az;





	DataStruct->Gyro_X_RAW = (int16_t)((BMI160_rxData[2]  << 8) | BMI160_rxData[1]);
	DataStruct->Gyro_Y_RAW = (int16_t)((BMI160_rxData[4]  << 8) | BMI160_rxData[3]);
	DataStruct->Gyro_Z_RAW = (int16_t)((BMI160_rxData[6]  << 8) | BMI160_rxData[5]);


	DataStruct->Gx = DataStruct->Gyro_X_RAW / 262.4;
	bmi160_gyro->x = DataStruct->Gx - BMI_GX_BIAS;

	DataStruct->Gy = DataStruct->Gyro_Y_RAW / 262.4;
	bmi160_gyro->y = DataStruct->Gy - BMI_GY_BIAS;

	DataStruct->Gz = DataStruct->Gyro_Z_RAW / 262.4;
	bmi160_gyro->z = DataStruct->Gz - BMI_GZ_BIAS;


	//stop_bmi = DWT->CYCCNT;
	//us_bmi = (stop_bmi - start_bmi) / (SystemCoreClock / 1000000.0f); // convert to microseconds





	// Kalman angle solve
	float dt = 0.0025;//(float)(HAL_GetTick() - bmi160_timer) / 1000;
	//bmi160_timer = HAL_GetTick();

						//gyroroll += dt * BMI_GX;
						//gyropitch += dt * BMI_GY;


	float roll;
	//start_bmi = DWT->CYCCNT;
	float roll_sqrt = sqrt(DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
	//stop_bmi = DWT->CYCCNT;
		    //us_bmi = (stop_bmi - start_bmi) / (SystemCoreClock / 1000000.0f); // convert to microseconds
	if (roll_sqrt != 0.0)
	{
		//start_bmi = DWT->CYCCNT;
	        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
			//roll = asin(BMI_AX - 0.05)* RAD_TO_DEG;
	        //accroll = roll;

	        //stop_bmi = DWT->CYCCNT;
	        		    //us_bmi = (stop_bmi - start_bmi) / (SystemCoreClock / 1000000.0f); // convert to microseconds

	}
	else
	{
	        roll = 0.0;
	}

	//start_bmi = DWT->CYCCNT;
	float pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
	//float pitch = atan2(-BMI_AY+0.04, -BMI_AZ-0.02)* RAD_TO_DEG;
	//stop_bmi = DWT->CYCCNT;
	//us_bmi = (stop_bmi - start_bmi) / (SystemCoreClock / 1000000.0f); // convert to microseconds
	//accpitch = pitch;

	if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
	{
	        bmi160_KalmanY.angle = pitch;
	        DataStruct->KalmanAngleY = pitch;
	}
	else
	{
		//start_bmi = DWT->CYCCNT;
	     DataStruct->KalmanAngleY = Kalman_getAngle1(&bmi160_KalmanY, pitch, bmi160_gyro->y, dt);
	     //stop_bmi = DWT->CYCCNT;
	     //us_bmi = (stop_bmi - start_bmi) / (SystemCoreClock / 1000000.0f); // convert to microseconds

	}

	//start_bmi = DWT->CYCCNT;
	if (fabs(DataStruct->KalmanAngleY) > 90)
	     DataStruct->Gx = -DataStruct->Gx;
	//stop_bmi = DWT->CYCCNT;
	//us_bmi = (stop_bmi - start_bmi) / (SystemCoreClock / 1000000.0f); // convert to microseconds
	DataStruct->KalmanAngleX = Kalman_getAngle1(&bmi160_KalmanX, roll, bmi160_gyro->x, dt);


	bmi_roll = (float)DataStruct->KalmanAngleX;
	bmi_pitch = (float)DataStruct->KalmanAngleY;



}



float Kalman_getAngle1(bmi160_Kalman_t *Kalman, float newAngle, float newRate, float dt)
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









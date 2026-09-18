#include <qmc5883.h>
#include <main.h>
#include <stdio.h>
#include <math.h>
#include "stm32h7xx_hal.h"


#define RAD_TO_DEG 57.295779513f
extern I2C_HandleTypeDef hi2c3;
const uint16_t hmc5883l_i2c_timeout = 100;
extern Struct_QMC5883P QMC5883P;

uint8_t CHECK, test_val;
uint8_t Data;
HAL_StatusTypeDef status;

float heading;
uint32_t start_qmc, stop_qmc;
float us_qmc;

float QMC_MX, QMC_MY, QMC_MZ;
float QMC_MX_1, QMC_MY_1, QMC_MZ_1;
float QMC_MX_BIAS, QMC_MY_BIAS, QMC_MZ_BIAS;			//BIAS is calculated by taking 200 samples then averaging them
//float icm160_timer;
float Y_QMC_MX_PREV, Y_QMC_MX_NEW, Y_QMC_MY_PREV, Y_QMC_MY_NEW,Y_QMC_MZ_PREV, Y_QMC_MZ_NEW;	//final values from the sensor(angular velocity)
float MX,MY,MZ;

uint8_t QMC_MAG_rxData[6];

void QMC5883P_Initialization(I2C_HandleTypeDef *I2Cx)
{
//	status = HAL_I2C_IsDeviceReady(
//	            &hi2c3,
//	            (0x2C  <<1 ),   // device address shifted left
//	            3,             // number of trials (try 3 times)
//	            10             // timeout per trial = 10ms
//	        );
//	if(status == HAL_OK)
//		Data++;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(&hi2c3, 0x2C<<1  , 0x00, I2C_MEMADD_SIZE_8BIT, &CHECK, 1, hmc5883l_i2c_timeout);


    HAL_StatusTypeDef status;

    // Write Control Register 1
    status = HAL_I2C_Mem_Write(&hi2c3, 0x2C<<1, QMC5883P_REG_CONTROL1, I2C_MEMADD_SIZE_8BIT, QMC5883P_REG_CONTROL1_VALUE, 1, 100);

    if (status != HAL_OK)
    {
       return; // handle error
    }

    // Small delay (optional)
    HAL_Delay(10);

    // Write Control Register 2
    status = HAL_I2C_Mem_Write(&hi2c3, 0x2C<<1, QMC5883P_REG_CONTROL2, I2C_MEMADD_SIZE_8BIT, QMC5883P_REG_CONTROL2_VALUE, 1, 100);

    if (status != HAL_OK)
    {
        return; // handle error
    }


   // HAL_I2C_Mem_Read_DMA(&hi2c3, 0x2C<<1  , 0x01, I2C_MEMADD_SIZE_8BIT, QMC_MAG_rxData, 6);

}







void QMC5883P_Get3AxisGyroRawData(Struct_QMC5883P *DataStruct, uint8_t *rxData)
{

	//start_icm = DWT->CYCCNT;
	//stop_icm = DWT->CYCCNT;
	//us_icm = (stop_icm - start_icm) / (SystemCoreClock / 1000000.0f); // convert to microseconds


	//uint8_t rxData[6];

	//txData[0] = GYRO_DATA_X1_UI | 0x80;  // set MSB = 1 for read
	//txData[1] = txData[2] = txData[3] = txData[4] = txData[5] = txData[6] = 0x00;


	start_qmc = DWT->CYCCNT;

	//HAL_I2C_Mem_Read(&hi2c3, 0x2C<<1  , 0x01, I2C_MEMADD_SIZE_8BIT, rxData, 6, hmc5883l_i2c_timeout);
	 //HAL_I2C_Mem_Read_DMA(&hi2c3, 0x2C<<1  , 0x01, I2C_MEMADD_SIZE_8BIT, rxData, 6);


	//HAL_I2C_Mem_Read_DMA(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);

	stop_qmc = DWT->CYCCNT;
	us_qmc = (stop_qmc - start_qmc) / (SystemCoreClock / 1000000.0f); // convert to microseconds


    DataStruct->Mag_X_RAW = (int16_t)((rxData[1] << 8) | rxData[0]);
    DataStruct->Mag_Y_RAW = (int16_t)((rxData[3] << 8) | rxData[2]);
    DataStruct->Mag_Z_RAW = (int16_t)((rxData[5] << 8) | rxData[4]);



    DataStruct->Mx = DataStruct->Mag_X_RAW / 3750.0;
    QMC_MX = DataStruct->Mx;

    DataStruct->My = DataStruct->Mag_Y_RAW / 3750.0;
    QMC_MY = DataStruct->My;

    DataStruct->Mz = DataStruct->Mag_Z_RAW / 3750.0;
    QMC_MZ = DataStruct->Mz;




//    ICM_GX = DataStruct->Gx - ICM_GX_BIAS;
//    ICM_GY = DataStruct->Gy - ICM_GY_BIAS;
//    ICM_GZ = DataStruct->Gz - ICM_GZ_BIAS;




    heading = atan2(QMC_MY, QMC_MX)* RAD_TO_DEG;

    if (heading < 0)
        heading += 360.0;

    //LOW-PASS FILTERING
//  double dt = 0.000625;  //the ODR = 1600Hz so the sampel period = 0.000625
//  double ICM_FREQ = 1.0 / dt;	//this the the ODR
//  double RC = (1/(2*3.14*100));	//RC is the time constant where 100 in the denominator is the cut-off frequency
//  double alpha = dt/(dt+RC);		//alpha is the smoothing coefficient


//    float alpha = 0.3;
//
//    //IIR_LPF_GX
//    Y_ICM_GX_NEW = (1-alpha)*Y_ICM_GX_PREV + alpha*ICM_GX;
//    Y_ICM_GX_PREV = Y_ICM_GX_NEW;
//
//    //IIR_LPF_GY
//    Y_ICM_GY_NEW = (1-alpha)*Y_ICM_GY_PREV + alpha*ICM_GY;
//    Y_ICM_GY_PREV = Y_ICM_GY_NEW;
//
//    //IIR_LPF_GZ
//    Y_ICM_GZ_NEW = (1-alpha)*Y_ICM_GZ_PREV + alpha*ICM_GZ;
//    Y_ICM_GZ_PREV = Y_ICM_GZ_NEW;
}

#ifndef __ICM45686_H
#define __ICM45686_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <axis3f.h>

// ==== Chip Select Macros ====
// ⚠️ Update GPIO port/pin according to your CubeMX config
#define ICM45686_CS_GPIO_PORT   GPIOC
#define ICM45686_CS_PIN         GPIO_PIN_4
#define ICM45686_CS_LOW()   HAL_GPIO_WritePin(ICM45686_CS_GPIO_PORT, ICM45686_CS_PIN, GPIO_PIN_RESET)
#define ICM45686_CS_HIGH()  HAL_GPIO_WritePin(ICM45686_CS_GPIO_PORT, ICM45686_CS_PIN, GPIO_PIN_SET)



//some definitions for the LL driver
#define ICM45686_SPI_CHANNEL		SPI1

#define ICM45686_SPI_SCLK_PIN		LL_GPIO_PIN_5
#define ICM45686_SPI_SCLK_PORT		GPIOA
#define ICM45686_SPI_SCLK_CLK		LL_AHB4_GRP1_PERIPH_GPIOA

#define ICM45686_SPI_MISO_PIN		LL_GPIO_PIN_6
#define ICM45686_SPI_MISO_PORT		GPIOA
#define ICM45686_SPI_MISO_CLK		LL_AHB4_GRP1_PERIPH_GPIOA

#define ICM45686_SPI_MOSI_PIN		LL_GPIO_PIN_7
#define ICM45686_SPI_MOSI_PORT		GPIOA
#define ICM45686_SPI_MOSI_CLK		LL_AHB4_GRP1_PERIPH_GPIOA

#define ICM45686_SPI_CS_PIN			LL_GPIO_PIN_4
#define ICM45686_SPI_CS_PORT		GPIOC
#define ICM45686_SPI_CS_CLK			LL_AHB1_GRP1_PERIPH_GPIOC
//
//#define ICM45686_INT_PIN			LL_GPIO_PIN_5
//#define ICM45686_INT_PORT			GPIOC
//#define ICM45686_INT_CLK			LL_AHB1_GRP1_PERIPH_GPIOC





// ==== Register Map (minimal for now) ====
#define ICM45686_REG_WHO_AM_I   0x72
#define ICM45686_WHO_AM_I_VALUE 0xE9   // Expected value (check datasheet to confirm)

// ==== External SPI Handle ====
// This must match your CubeMX SPI instance (e.g., hspi1)
//extern SPI_HandleTypeDef hspi1;

typedef struct _ICM45686{
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	short temperature_raw;
	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;

	float Ax;
	float Ay;
	float Az;
	float Gx;
	float Gy;
	float Gz;

	float KalmanAngleX;
	float KalmanAngleY;

}Struct_ICM45686;



typedef struct
{
	float Q_angle;
	float Q_bias;
	float R_measure;
	float angle;
	float bias;
	float P[2][2];
} icm45686_Kalman_t;




/* DREG_BANK1 */
#define ACCEL_DATA_X1_UI                                                        0x00
#define ACCEL_DATA_X0_UI                                                        0x01
#define ACCEL_DATA_Y1_UI                                                        0x02
#define ACCEL_DATA_Y0_UI                                                        0x03
#define ACCEL_DATA_Z1_UI                                                        0x04
#define ACCEL_DATA_Z0_UI                                                        0x05
#define GYRO_DATA_X1_UI                                                         0x06
#define GYRO_DATA_X0_UI                                                         0x07
#define GYRO_DATA_Y1_UI                                                         0x08
#define GYRO_DATA_Y0_UI                                                         0x09
#define GYRO_DATA_Z1_UI                                                         0x0a
#define GYRO_DATA_Z0_UI                                                         0x0b
#define TEMP_DATA1_UI                                                           0x0c
#define TEMP_DATA0_UI                                                           0x0d
#define TMST_FSYNCH                                                             0x0e
#define TMST_FSYNCL                                                             0x0f
#define PWR_MGMT0                                                               0x10
#define FIFO_COUNT_0                                                            0x12
#define FIFO_COUNT_1                                                            0x13
#define FIFO_DATA                                                               0x14
#define INT1_CONFIG0                                                            0x16
#define INT1_CONFIG1                                                            0x17
#define INT1_CONFIG2                                                            0x18
#define INT1_STATUS0                                                            0x19
#define INT1_STATUS1                                                            0x1a
#define ACCEL_CONFIG0                                                           0x1b
#define GYRO_CONFIG0                                                            0x1c


#define PWR_MGMT0_VALUE					0x0f
#define ACCEL_CONFIG0_VALUE             0x45
#define GYRO_CONFIG0_VALUE 				0x45
#define INT1_CONFIG0_VALUE              0x04
#define INT1_CONFIG1_VALUE              0x00
#define INT1_CONFIG2_VALUE              0x01
//#define INT1_STATUS0_VALUE


// ==== Public API ====
uint8_t ICM45686_ReadReg(uint8_t reg);
void ICM45686_WriteReg(uint8_t reg, uint8_t value);
void ICM45686_Initialization();
//void ICM45686_Get3AxisGyroRawData(Struct_ICM45686 *DataStruct);
void ICM45686_Get3AxisGyroRawData(Struct_ICM45686 *DataStruct, Axis3f *icm45686_gyro, Axis3f *icm45686_acc, Axis3f_Bias *gyro_bias);
float ICM45686_Kalman_getAngle1(icm45686_Kalman_t *Kalman, float newAngle, float newRate, float dt);


#ifdef __cplusplus
}
#endif

#endif /* __ICM45686_H */

/*
 * BMI160.H
 *
 *  Created on: Sep 21, 2025
 *      Author: SAIED
 */

#ifndef INC_BMI160_H_
#define INC_BMI160_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <axis3f.h>

// ==== Chip Select Macros ====
// ⚠️ Update GPIO port/pin according to your CubeMX config
#define BMI160_CS_GPIO_PORT   GPIOB
#define BMI160_CS_PIN         GPIO_PIN_11

#define BMI160_INT1_GPIO_PORT   GPIOE
#define BMI160_INT1_PIN         GPIO_PIN_14



#define BMI160_CS_LOW()   HAL_GPIO_WritePin(BMI160_CS_GPIO_PORT, BMI160_CS_PIN, GPIO_PIN_RESET)
#define BMI160_CS_HIGH()  HAL_GPIO_WritePin(BMI160_CS_GPIO_PORT, BMI160_CS_PIN, GPIO_PIN_SET)

//#define BMI160_INT1_LOW()   HAL_GPIO_WritePin(BMI160_INT1_GPIO_PORT, BMI160_INT1_PIN, GPIO_PIN_RESET)
//#define BMI160_INT1_HIGH()  HAL_GPIO_WritePin(BMI160_INT1_GPIO_PORT, BMI160_INT1_PIN, GPIO_PIN_SET)

//Register Address
#define CHIPID   			0x00

#define PMU_STATUS			0x03

#define GYR_X_L 			0x0C
#define GYR_X_H 			0x0D
#define GYR_Y_L 			0x0E
#define GYR_Y_H 			0x0F
#define GYR_Z_L 			0x10
#define GYR_Z_H 			0x11
#define ACC_X_L 			0x12
#define ACC_X_H 			0x13
#define ACC_Y_L 			0x14
#define ACC_Y_H 			0x15
#define ACC_Z_L 			0x16
#define ACC_Z_H 			0x17

#define ACC_CONF			0x40
#define ACC_RANGE			0x41
#define GYR_CONF			0x42
#define GYR_RANGE			0x43

#define CMD					0x7E

//Register Value
#define PMU_STATUS_VALUE		0x04
#define CHIPID_VALUE 			0xD1 		 // Expected value (check datasheet to confirm)
#define ACC_CONF_VALUE			0x18
#define ACC_RANGE_VALUE			0x03
#define GYR_CONF_VALUE			0x2A
#define GYR_RANGE_VALUE			0x04
#define CMD_VALUE				0x15






// ==== External SPI Handle ====
// This must match your CubeMX SPI instance (e.g., hspi1)
extern SPI_HandleTypeDef hspi2;


/**
 * @brief BMI160 structure definition.
 */

typedef struct _BMI160{
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

}Struct_BMI160;

typedef struct
{
	float Q_angle;
	float Q_bias;
	float R_measure;
	float angle;
	float bias;
	float P[2][2];
} bmi160_Kalman_t;

/**
 * @brief BMI160 structure definition.
 */

extern Struct_BMI160 BMI160;



// ==== Public API ====
void 				BMI160_WriteReg(uint8_t reg, uint8_t value);
uint8_t 			BMI160_ReadReg(uint8_t reg);
void				BMI160_Initialization1(void);
void 				BMI160_Get6AxisRawData(Struct_BMI160 *DataStruct, Axis3f *bmi160_gyro, Axis3f *bmi160_acc);
void 				BMI160_Get3AxisGyroRawData(Struct_BMI160 *DataStruct);
void		 		BMI160_Get3AxisAccRawData(Struct_BMI160 *DataStruct);
float 				Kalman_getAngle1(bmi160_Kalman_t *Kalman, float newAngle, float newRate, float dt);
//void 				HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif




#endif /* INC_BMI160_H_ */

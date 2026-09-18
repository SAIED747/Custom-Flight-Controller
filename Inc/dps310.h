/*
 * dps310.h
 *
 *  Created on: Oct 7, 2025
 *      Author: SAIED
 */

#ifndef INC_DPS310_H_
#define INC_DPS310_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

extern SPI_HandleTypeDef hspi3;


typedef struct _DPS310{
	uint32_t Temp_Raw;
	uint32_t Pres_Raw;

	int32_t Temp_Value;
	int32_t Pres_Value;

	float Temp_Value_Scaled;
	float Pres_Value_Scaled;

	float Temp_Compensated;
	float Pres_Compensated;
}Struct_DPS310;



// ==== Chip Select Macros ====
// ⚠️ Update GPIO port/pin according to your CubeMX config
#define DPS310_CS_GPIO_PORT   GPIOB
#define DPS310_CS_PIN         GPIO_PIN_1

#define DPS310_CS_LOW()   HAL_GPIO_WritePin(DPS310_CS_GPIO_PORT, DPS310_CS_PIN, GPIO_PIN_RESET)
#define DPS310_CS_HIGH()  HAL_GPIO_WritePin(DPS310_CS_GPIO_PORT, DPS310_CS_PIN, GPIO_PIN_SET)


#define DPS310_CHIPID  			0x0D

#define PSR_B2					0x00
#define PSR_B1					0x01
#define PSR_B0					0x02
#define TMP_B2					0x03
#define TMP_B1					0x04
#define TMP_B0					0x05

#define PRS_CFG					0x06
#define TMP_CFG					0x07
#define MEAS_CFG				0x08
#define CFG_REG					0x09
#define INT_STS					0x0A
#define FIFO_STS				0x0B
#define RESET					0x0C

#define CAL_COEF_0				0x10
#define CAL_COEF_1				0x11
#define CAL_COEF_2				0x12
#define CAL_COEF_3				0x13
#define CAL_COEF_4				0x14
#define CAL_COEF_5				0x15
#define CAL_COEF_6				0x16
#define CAL_COEF_7				0x17
#define CAL_COEF_8				0x18
#define CAL_COEF_9				0x19
#define CAL_COEF_10				0x1A
#define CAL_COEF_11				0x1B
#define CAL_COEF_12				0x1C
#define CAL_COEF_13				0x1D
#define CAL_COEF_14				0x1E
#define CAL_COEF_15				0x1F
#define CAL_COEF_16				0x20
#define CAL_COEF_17				0x21

#define TMP_COEF_SRCE			0x28





#define DPS310_CHIPID_VALUE		0x10
#define PRS_CFG_VALUE			0x33//0x66//0x33
#define TMP_CFG_VALUE			0xB3//0xE6//0xB3
#define CFG_REG_VALUE			0xB0//0xBC//0xB0




void 					DPS310_WriteReg(uint8_t reg, uint8_t value);
uint8_t 				DPS310_ReadReg(uint8_t reg);
static inline int32_t 	twos_complement (const uint32_t value, const uint8_t bits);
void					DPS310_Initialization(void);
void		 			DPS310_GetTempPres(Struct_DPS310 *DataStruct);

#endif /* INC_DPS310_H_ */

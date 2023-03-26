/*
 * LIS2DW.h
 *
 *  Created on: Dec 4, 2022
 *      Author: omirza
 */

#ifndef LIS2DW_H
#define LIS2DW_H


#include "stm32wbxx_hal.h" // Needed for I2C
#include "stm32wbxx_hal_gpio.h" // Needed for I2C
/*
 * I2C address and Register defines
 */
// I2C address pg. 28
#define LIS2DW_I2C_ADDRr	0x33 // Read

// WHO AM I ID
#define LIS2DW_DEVID 		0x44

// Register defines pg. 33
#define TEMP_12BIT_LOW	    0X0D //2s compliment 12-bit
#define TEMP_12BIT_HIGH	    0x0E //2s compliment 12-bit
#define CHIP_ID 			0x0f
#define CTRL1_REG			0x20
#define CTRL2_REG			0x21
#define CTRL3_REG			0x22
#define CTRL4_REG			0x23
#define CTRL5_REG			0x24
#define CTRL6_REG			0x25
#define TEMP_8BIT			0x26
#define STATUS__DATA_REG	0x27
#define ACC_OUT_X_L			0x28
#define ACC_OUT_X_H			0x29
#define ACC_OUT_Y_L			0x2A
#define ACC_OUT_Y_H			0x2B
#define ACC_OUT_Z_L			0x2C
#define ACC_OUT_Z_H			0x2D
#define FIFO_CTRL			0x2E
#define FIFO_SAMPLES		0x2F
#define TAP_THS_X			0x30
#define TAP_THS_Y			0x31
#define TAP_THS_Z			0x32
#define INT_DUR				0x33
#define WKUP_THS			0x34
#define WKUP_DUR			0x35
#define FREE_FALL			0x36
#define STATUS_DUP			0x37
#define WKUP_SRC			0x38
#define TAP_SRC				0x39
#define SIXD_SRC			0x3A
#define ALL_INT_SRC			0x3B
#define X_OFF_USR			0x3C
#define Y_OFF_USR			0x3D
#define Z_OFF_USR			0x3E
#define CTRL7_REG			0x3F

// LED Debug Pin Def

#define GREEN_Pin GPIO_PIN_0
#define GREEN_GPIO_Port GPIOB
#define RED_Pin GPIO_PIN_1
#define RED_GPIO_Port GPIOB
#define BLUE_Pin GPIO_PIN_5
#define BLUE_GPIO_Port GPIOB

typedef struct {

	/* I2C Handle */
	I2C_HandleTypeDef *i2cHandle;

	float accel_mps2[3];

	float temp_C;
} LIS2DW;

/* Function Definitions */
// Accelerometer Initialization

uint8_t LIS2DW_Init( LIS2DW *dev, I2C_HandleTypeDef *i2cHandle);
// Accelerometer data read

HAL_StatusTypeDef LIS2DW_ReadTemp(LIS2DW *dev);
HAL_StatusTypeDef LIS2DW_ReadAccel(LIS2DW *dev);

HAL_StatusTypeDef LIS2DW_ReadRegister(LIS2DW *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef LIS2DW_ReadRegisters(LIS2DW *dev, uint8_t reg, uint8_t *data, uint8_t length);
HAL_StatusTypeDef LIS2DW_WriteRegister(LIS2DW *dev, uint8_t reg, uint8_t *data);


//


#endif /* INC_LIS2DW_H_ */

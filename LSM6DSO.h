/*
 * LSM6DSO.h
 *
 *  Created on: Jan 21, 2023
 *      Author: omirza
 */

#ifndef INC_LSM6DSO_H_
#define INC_LSM6DSO_H_

#include "stm32wbxx_hal.h" // Needed for I2C
#include "stm32wbxx_hal_gpio.h" // Needed for I2C

#define LSM6DSO_I2C_ADDRr	0xD6 // Read, DSO bit 0 pulled high (7-bit Slave Address)
#define DEV_ID_IMU			0x6C

#define	FUNC_CFG_ACCESS 	0x01
#define	PIN_CTRL			0x02
//#define	RESERVED
#define	FIFO_CTRL1 			0x07
#define	FIFO_CTRL2 			0x08
#define	FIFO_CTRL3 			0x09
#define	FIFO_CTRL4 			0x0A
#define	COUNTER_BDR_REG1 	0x0B
#define	COUNTER_BDR_REG2 	0x0C
#define	INT1_CTRL 			0x0D
#define	INT2_CTRL 			0x0E
#define	WHO_AM_I			0x0F // Device ID = 0x6C
#define	CTRL1_XL			0x10 // Accelerometer control register 1
#define	CTRL2_G				0x11 // Gyroscope control register 2
#define	CTRL3_C				0x12
#define	CTRL4_C				0x13
#define	CTRL5_C				0x14
#define	CTRL6_C				0x15
#define	CTRL7_G				0x16
#define	CTRL8_XL			0x17
#define	CTRL9_XL			0x18
#define	CTRL10_C			0x19
#define	ALL_INT_SRC_IMU		0x1A
#define	WAKE_UP_SRC			0x1B
#define	TAP_SRC_IMU			0x1C
#define	D6D_SRC				0x1D
#define	STATUS_REG 			0x1E
//#define	RESERVED			0x1F
#define	OUT_TEMP_L 			0x20
#define	OUT_TEMP_H 			0x21

// Gyroscope Output Registers - 2's compliment, 16 bit (8bit H, 8bit L)
#define	OUTX_L_G 			0x22
#define	OUTX_H_G 			0x23
#define	OUTY_L_G			0x24
#define	OUTY_H_G			0x25
#define	OUTZ_L_G			0x26
#define	OUTZ_H_G			0x27

// Accelerometer Output Registers - 2's compliment, 16 bit (8bit H, 8bit L)
#define	OUTX_L_A			0x28
#define	OUTX_H_A			0x29
#define	OUTY_L_A			0x2A
#define	OUTY_H_A			0x2B
#define	OUTZ_L_A			0x2C
#define	OUTZ_H_A			0x2D
//#define	RESERVED
#define	EMB_FUNC_STATUS_MAINPAGE 	0x35
#define	FSM_STATUS_A_MAINPAGE 		0x36
#define	FSM_STATUS_B_MAINPAGE 		0x37
//#define	RESERVED
#define	STATUS_MASTER_MAINPAGE 		0x39
#define	FIFO_STATUS1				0x3A
#define	FIFO_STATUS2 				0x3B
//#define	RESERVED
#define	TIMESTAMP0 			0x40
#define	TIMESTAMP1 			0x41
#define	TIMESTAMP2 			0x42
#define	TIMESTAMP3 			0x43
//#define	RESERVED
#define	TAP_CFG0			0x56
#define	TAP_CFG1			0x57
#define	TAP_CFG2 			0x58
#define	TAP_THS_6D 			0x59
#define	INT_DUR2 			0x5A
#define	WAKE_UP_THS 		0x5B
#define	WAKE_UP_DUR 		0x5C
#define	FREE_FALL_IMU		0x5D
#define	MD1_CFG				0x5E
#define	MD2_CFG				0x5F
//#define	RESERVED
#define	I3C_BUS_AVB 		0x62
#define	INTERNAL_FREQ_FINE 	0x63
#define	RESERVED
#define	INT_OIS 			0x6F
#define	CTRL1_OIS			0x70
#define	CTRL2_OIS 			0x71
#define	CTRL3_OIS 			0x72
#define	X_OFS_USR 			0x73
#define	Y_OFS_USR 			0x74
#define	Z_OFS_USR 			0x75
//#define	RESERVED
#define	FIFO_DATA_OUT_TAG 	0x78
#define	FIFO_DATA_OUT_X_L 	0x79
#define	FIFO_DATA_OUT_X_H 	0x7A
#define	FIFO_DATA_OUT_Y_L 	0x7B
#define	FIFO_DATA_OUT_Y_H 	0x7C
#define	FIFO_DATA_OUT_Z_L 	0x7D
#define	FIFO_DATA_OUT_Z_H	0x7E

typedef struct {

	/* I2C Handle */
	I2C_HandleTypeDef *i2cHandle;

	float accel_mps2[3];	// 3 axis accel data array

	float gyro_dps[3];		// 3 axis gyro data array

	float temp_C;			// Temp Sensor
} LSM6DSO;

/* Function Definitions */
// Accelerometer Initialization

uint8_t LSM6DSO_Init( LSM6DSO *dev, I2C_HandleTypeDef *i2cHandle);
// Accelerometer data read

HAL_StatusTypeDef LSM6DSO_ReadTemp(LSM6DSO *dev);
HAL_StatusTypeDef LSM6DSO_ReadAccel(LSM6DSO *dev);
HAL_StatusTypeDef LSM6DSO_ReadGyro(LSM6DSO *dev);
HAL_StatusTypeDef LSM6DSO_ReadRegister(LSM6DSO *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef LSM6DSO_ReadRegisters(LSM6DSO *dev, uint8_t reg, uint8_t *data, uint8_t length);
HAL_StatusTypeDef LSM6DSO_WriteRegister(LSM6DSO *dev, uint8_t reg, uint8_t *data);

#endif /* INC_LSM6DSO_H_ */

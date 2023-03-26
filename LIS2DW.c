/*
 * LIS2DW.c
 *
 *  Created on: Dec 4, 2022
 *      Author: omirza
 */
#include "LIS2DW.h"

// Low level function
HAL_StatusTypeDef LIS2DW_ReadRegister(LIS2DW *dev, uint8_t reg, uint8_t *data) {

	return HAL_I2C_Mem_Read(dev->i2cHandle, LIS2DW_I2C_ADDRr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef LIS2DW_ReadRegisters(LIS2DW *dev, uint8_t reg, uint8_t *data, uint8_t length) {

	return HAL_I2C_Mem_Read(dev->i2cHandle, LIS2DW_I2C_ADDRr, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);

}

HAL_StatusTypeDef LIS2DW_WriteRegister(LIS2DW *dev, uint8_t reg, uint8_t *data) {

	return HAL_I2C_Mem_Write(dev->i2cHandle, LIS2DW_I2C_ADDRr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
// Sensor initialize function
uint8_t LIS2DW_Init(LIS2DW *dev, I2C_HandleTypeDef *i2cHandle){

	dev->i2cHandle = i2cHandle;

	dev->accel_mps2[0]		= 0.0f;
	dev->accel_mps2[1]		= 0.0f;
	dev->accel_mps2[2]		= 0.0f;

	dev->temp_C		= 0.0f;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	//Check device ID
	uint8_t regData;

	status = LIS2DW_ReadRegister(dev, CHIP_ID, &regData);
	errNum += (status != HAL_OK);

	if(regData != LIS2DW_DEVID) {
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_SET);
		errNum = 255;
	}
	else {
	HAL_GPIO_WritePin(GPIOB, BLUE_Pin , GPIO_PIN_SET);
	}

	// Set ODR 800 Hz, High-Performance, LPM3
	regData = 0x96;
	status = LIS2DW_WriteRegister(dev, CTRL1_REG, &regData);
	errNum += (status != HAL_OK);

	status = LIS2DW_ReadRegister(dev, CTRL1_REG, &regData);
	errNum += (status != HAL_OK);

	// Set BW 800/2 Hz, +/-4 FS, Low Noise Disable
	regData = 0x10;
	status = LIS2DW_WriteRegister(dev, CTRL6_REG, &regData);
	errNum += (status != HAL_OK);

	status = LIS2DW_ReadRegister(dev, CTRL6_REG, &regData);
	errNum += (status != HAL_OK);

	// 0 errors means initialization was successful

	return errNum;
};
// Accelerometer and Temperature data read

HAL_StatusTypeDef LIS2DW_ReadTemp(LIS2DW *dev) {
	int8_t regData[2];

	HAL_StatusTypeDef status = LIS2DW_ReadRegisters(dev, TEMP_12BIT_LOW, regData, 2);
	int16_t tempRaw2c = ((regData[0]>>4 & 0x0F) | regData[1]<<4);

	/* Convert rawTemp to deg C
	 *0 LSB = 25˚C
	 *Find Temp < or > 0˚C     */
	/*if((tempRaw2c & 0x0800) == 0x0800){
		int16_t tempRaw = (((~tempRaw2c) + 1) & 0x0FFF);
		dev->temp_C = (25.0f - (float) tempRaw * 0.0625f);
	}
	else{*/
	int16_t tempRaw = tempRaw2c;
	dev->temp_C = (25.0f + (float) tempRaw * 0.0625f);
	//}

	if((dev->temp_C )> 25){
		HAL_GPIO_WritePin(GPIOB, RED_Pin , GPIO_PIN_SET);
	  }
	return status;
}

HAL_StatusTypeDef LIS2DW_ReadAccel(LIS2DW *dev) {

	int8_t regData[6];
	HAL_StatusTypeDef status = LIS2DW_ReadRegisters(dev, ACC_OUT_X_L, regData, 6);
	// X,Y,Z data are 14 bits, 2s compliment, 8 High bits, 6 (MSB) Low bits

	int16_t rawAccel_x = ((regData[0] & 0xFC) | (regData[1]<<8))>>2;  // X-Axis data
	int16_t rawAccel_y = ((regData[2] & 0xFC) | (regData[3]<<8))>>2;  // Y-Axis data
	int16_t rawAccel_z = ((regData[4] & 0xFC) | (regData[5]<<8))>>2;  // Z-Axis data


	dev->accel_mps2[0] = rawAccel_x * 0.00048828125f *9.81f;
	dev->accel_mps2[1] = rawAccel_y * 0.00048828125f *9.81f;
	dev->accel_mps2[2] = rawAccel_z * 0.00048828125f *9.81f;

	return status;
}


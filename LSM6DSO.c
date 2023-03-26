/*
 * LSM6DSO.c
 *
 *  Created on: Jan 25, 2023
 *      Author: omirza
 */

#include "LSM6DSO.h"

HAL_StatusTypeDef LSM6DSO_ReadRegister(LSM6DSO *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Read(dev->i2cHandle, LSM6DSO_I2C_ADDRr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef LSM6DSO_ReadRegisters(LSM6DSO *dev, uint8_t reg, uint8_t *data, uint8_t length){

	return HAL_I2C_Mem_Read(dev->i2cHandle, LSM6DSO_I2C_ADDRr, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);

}

HAL_StatusTypeDef LSM6DSO_WriteRegister(LSM6DSO *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Write(dev->i2cHandle, LSM6DSO_I2C_ADDRr, reg, I2C_MEMADD_SIZE_8BIT, data,1, HAL_MAX_DELAY);
}

uint8_t LSM6DSO_Init(LSM6DSO *dev, I2C_HandleTypeDef *i2cHandle){

	dev->i2cHandle = i2cHandle;

	// Initialize Accel/Gyro and Temp sensor data arrays to 0
	dev->accel_mps2[0] = 	0.0f;
	dev->accel_mps2[1] = 	0.0f;
	dev->accel_mps2[2] = 	0.0f;


	dev->gyro_dps[0]   = 	0.0f;
	dev->gyro_dps[1]   = 	0.0f;
	dev->gyro_dps[2]   =    0.0f;

	dev->temp_C		   = 	0.0f;

	// Initialize error counter, each write/read function will output error status
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint8_t regData;

	// Check deviced ID
	status = LSM6DSO_ReadRegister(dev, WHO_AM_I, &regData);
	errNum += (status != HAL_OK);

	if(regData != DEV_ID_IMU){
		errNum = 255;
	}
	// Write Accel config register
	regData = 0x48; //4g, 26hz
	status = LSM6DSO_WriteRegister(dev, CTRL1_XL, &regData);
	errNum += (status != HAL_OK);
	status = LSM6DSO_ReadRegister(dev, CTRL1_XL, &regData);
	errNum += (status != HAL_OK);

	// Write Gyro config register
	regData = 0x64;
	status = LSM6DSO_WriteRegister(dev, CTRL2_G, &regData);
	errNum += (status != HAL_OK);
	status = LSM6DSO_ReadRegister(dev, CTRL2_G, &regData);
	errNum += (status != HAL_OK);

	return errNum;

};

HAL_StatusTypeDef LSM6DSO_ReadAccel(LSM6DSO *dev){

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint8_t regData[6];

	status = LSM6DSO_ReadRegisters(dev, OUTX_L_A, regData, 6);
	errNum += (status != HAL_OK);

	int16_t rawAccel_x = ((regData[1]<<8) | (regData[0]));
	int16_t rawAccel_y = ((regData[3]<<8) | (regData[2]));
	int16_t rawAccel_z = ((regData[5]<<8) | (regData[4]));

	dev->accel_mps2[0] = rawAccel_x*0.122/1000*9.81;
	dev->accel_mps2[1] = rawAccel_y*0.122/1000*9.81;
	dev->accel_mps2[2] = rawAccel_z*0.122/1000*9.81;


	return status;
};

HAL_StatusTypeDef LSM6DSO_ReadGyro(LSM6DSO *dev){

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint8_t regData[6];

	status = LSM6DSO_ReadRegisters(dev, OUTX_L_G, regData, 6);
	errNum += (status != HAL_OK);

	int16_t rawGyro_x = ((regData[1]<<8) | (regData[0]));
	int16_t rawGyro_y = ((regData[3]<<8) | (regData[2]));
	int16_t rawGyro_z = ((regData[5]<<8) | (regData[4]));

	dev->gyro_dps[0] = rawGyro_x*8.75/1000;
	dev->gyro_dps[1] = rawGyro_y*8.75/1000;
	dev->gyro_dps[2] = rawGyro_z*8.75/1000;

	return status;

};

HAL_StatusTypeDef LSM6DSO_ReadTemp(LSM6DSO *dev){

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint8_t regData[2];

	status = LSM6DSO_ReadRegisters(dev, OUT_TEMP_L, regData, 2);
	errNum += (status != HAL_OK);

	int16_t rawTemp = ((regData[1]<<8) | (regData[0]));
	// 256 LSB/˚C, 0 LSB = 25˚C
	dev->temp_C = rawTemp/256.0f + 25.0f;

	return status;
};




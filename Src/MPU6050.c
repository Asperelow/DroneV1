/*
 * MPU6050 Accelerometer and Gyroscope driver for STM32f1xx
 *
 *  Created on: Jun 8, 2023
 *      Author: Skyler Horn
 */

#include "MPU6050.h"

void MPU6050_Init(I2C_HandleTypeDef *handler, MPU6050_t *dataStruct){
	uint8_t check, Data;
	dataStruct->rx = 0;

	HAL_I2C_Mem_Read(handler, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, HAL_MAX_DELAY);
	if(check == 104){
		Data = 0;
		// Power Management register 0X6B: Write all 0's to wake the sensor
		HAL_I2C_Mem_Write(handler, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, HAL_MAX_DELAY);
		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(handler, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, HAL_MAX_DELAY);

		// Set acc. config in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +-2 g
		Data = 0x00;
		HAL_I2C_Mem_Write(handler, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, HAL_MAX_DELAY);

		// Set gyro config in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +-250 dgps
		Data = 0x00;
		HAL_I2C_Mem_Write(handler, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, HAL_MAX_DELAY);

		Data = 0x10;
		HAL_I2C_Mem_Write(handler, MPU6050_ADDR, MPU_INT_CONFIG_REG, 1, &Data, 1, HAL_MAX_DELAY);
		Data = 0x01;
		HAL_I2C_Mem_Write(handler, MPU6050_ADDR, MPU_INT_ENABLE, 1, &Data, 1, HAL_MAX_DELAY);

	}
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *handler, MPU6050_t *dataStruct)
{
    uint8_t recData[6];

    HAL_I2C_Mem_Read(handler,MPU6050_ADDR,ACCEL_XOUT_H_REG,I2C_MEMADD_SIZE_8BIT,recData,6,100);
    HAL_Delay(50);

    dataStruct->Accel_RAW[0] = (int16_t)(recData[0] << 8 | recData[1]);
    dataStruct->Accel_RAW[1] = (int16_t)(recData[2] << 8 | recData[3]);
    dataStruct->Accel_RAW[2] = (int16_t)(recData[4] << 8 | recData[5]);

    // swapped up for some reason?
    dataStruct->Accel[0] = -(dataStruct->Accel_RAW[1] * MPU6050_ACC_RAW_TO_MPS2);
    dataStruct->Accel[1] = -(dataStruct->Accel_RAW[0] * MPU6050_ACC_RAW_TO_MPS2);
    dataStruct->Accel[2] = -(dataStruct->Accel_RAW[2] * MPU6050_ACC_RAW_TO_MPS2);
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *handler, MPU6050_t *dataStruct)
{
    uint8_t recData[6];

    HAL_I2C_Mem_Read(handler,MPU6050_ADDR,GYRO_XOUT_H_REG,I2C_MEMADD_SIZE_8BIT,recData,6,100);
    HAL_Delay(50);

    dataStruct->Gyro_RAW[0] = (int16_t)(recData[0] << 8 | recData[1]);
    dataStruct->Gyro_RAW[1] = (int16_t)(recData[2] << 8 | recData[3]);
    dataStruct->Gyro_RAW[2] = (int16_t)(recData[4] << 8 | recData[5]);

    // swapped up for some reason?
    dataStruct->Gyro[0] = -(dataStruct->Gyro_RAW[0] * MPU6050_GYR_RAW_TO_RPS);
    dataStruct->Gyro[1] = -(dataStruct->Gyro_RAW[1] * MPU6050_GYR_RAW_TO_RPS);
    dataStruct->Gyro[2] = -(dataStruct->Gyro_RAW[2] * MPU6050_GYR_RAW_TO_RPS);

}



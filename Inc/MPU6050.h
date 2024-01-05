/*
 * MPU6050 Accelerometer and Gyroscope driver for STM32f1xx
 *
 *  Created on: Jun 8, 2023
 *      Author: Skyler Horn
 */

#ifndef INC_MPU6050_H
#define INC_MPU6050_H

#include "stm32f1xx_hal.h"	// Required for I2C
#include "stm32f1xx_hal_i2c.h"	// Required for I2C


#define MPU6050_ADDR 0xD0

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19		// Sample Rate Divider
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

#define MPU_INT_CONFIG_REG 0x37
#define MPU_INT_ENABLE 0x38

#define MPU6050_ACC_RAW_TO_MPS2 0.00059875482f
#define MPU6050_GYR_RAW_TO_RPS	0.00013323124

typedef struct
{
    int16_t Accel_RAW[3];
    float Accel[3];

    int16_t Gyro_RAW[3];
    float Gyro[3];

    int rx;
} MPU6050_t;

void MPU6050_Init(I2C_HandleTypeDef *handler, MPU6050_t *dataStruct);
void MPU6050_Read_Accel(I2C_HandleTypeDef *handler, MPU6050_t *dataStruct);
void MPU6050_Read_Gyro(I2C_HandleTypeDef *handler, MPU6050_t *dataStruct);


#endif /* INC_MPU6050_H */

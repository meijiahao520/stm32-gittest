#ifndef MPU_6050_H_
#define MPU_6050_H_

#include "i2c.h"

#define MPU6050_ADDR 0xD0
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

typedef struct {
    int16_t x, y, z;
} AccelData;

typedef struct {
    int16_t x, y, z;
} GyroData;

extern volatile float roll, pitch, yaw;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c, AccelData *accel);
HAL_StatusTypeDef MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c, GyroData *gyro);
HAL_StatusTypeDef MPU6050_UpdateData(I2C_HandleTypeDef *hi2c, float *dt_out);

#endif /* MPU_6050_H_ */

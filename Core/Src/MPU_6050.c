#include "MPU_6050.h"
#include "math.h"

#ifndef M_PI
#define M_PI acos(-1.0)
#endif
// 添加全局变量用于存储角度和时间戳（假设在.h中声明）
volatilexx float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
static uint32_t lastUpdateTick = 0;
const float alpha = 0.98f;  // 互补滤波系数（可调整，0.98表示更信任陀螺仪）

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    // 检查设备是否就绪（非阻塞，单次检查）
    if (HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR, 3, 100) != HAL_OK) {
        return HAL_ERROR;  // 设备未就绪
    }
    // 唤醒MPU6050（写入0x00到PWR_MGMT_1寄存器）
    uint8_t data = 0x00;
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c, AccelData *accel) {
    uint8_t data[6];
    // 读取加速度计数据（6字节，从ACCEL_XOUT_H开始）
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    if (status != HAL_OK) {
        return status;  // 返回错误状态
    }
    accel->x = (data[0] << 8) | data[1];
    accel->y = (data[2] << 8) | data[3];
    accel->z = (data[4] << 8) | data[5];
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c, GyroData *gyro) {
    uint8_t data[6];
    // 读取陀螺仪数据（6字节，从GYRO_XOUT_H开始）
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    if (status != HAL_OK) {
        return status;  // 返回错误状态
    }
    gyro->x = (data[0] << 8) | data[1];
    gyro->y = (data[2] << 8) | data[3];
    gyro->z = (data[4] << 8) | data[5];
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_UpdateData(I2C_HandleTypeDef *hi2c) {
    AccelData accel;
    GyroData gyro;
    // 读取加速度计和陀螺仪数据
    if (MPU6050_ReadAccel(hi2c, &accel) != HAL_OK || MPU6050_ReadGyro(hi2c, &gyro) != HAL_OK) {
        return HAL_ERROR;  // 读取失败
    }
    // 用户处理代码区域：在此处添加对accel.x/y/z和gyro.x/y/z的处理逻辑  
    // 计算时间间隔（dt，单位秒）
    uint32_t currentTick = HAL_GetTick();
    float dt = (currentTick - lastUpdateTick) / 1000.0f;  // 转换为秒
    lastUpdateTick = currentTick;

    // 将原始数据转换为实际值（假设量程为±2g和±250°/s，可根据需要调整）
    float accelX = accel.x / 16384.0f;  // 加速度计灵敏度
    float accelY = accel.y / 16384.0f;
    float accelZ = accel.z / 16384.0f;
    float gyroX = gyro.x / 131.0f * M_PI / 180.0f;  // 陀螺仪灵敏度，转换为rad/s
    float gyroY = gyro.y / 131.0f * M_PI / 180.0f;
    float gyroZ = gyro.z / 131.0f * M_PI / 180.0f;

    // 从加速度计计算倾角（pitch和roll）
    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0f / M_PI;
    float accelRoll = atan2(accelY, accelZ) * 180.0f / M_PI;

    // 从陀螺仪积分计算角度（仅pitch和roll）
    pitch += gyroX * dt * 180.0f / M_PI;
    roll += gyroY * dt * 180.0f / M_PI;
    yaw += gyroZ * dt * 180.0f / M_PI;

    // 互补滤波融合（仅对pitch和roll）
    pitch = alpha * pitch + (1 - alpha) * accelPitch;
    roll = alpha * roll + (1 - alpha) * accelRoll;

    // 用户处理代码区域：在此处添加对最终欧拉角的处理逻辑
    // 例如：输出到串口、用于控制等
    

    return HAL_OK;
}

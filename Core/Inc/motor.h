#ifndef MOTOR_H_
#define MOTOR_H_

#include "tim.h"  // 假设使用TIM进行PWM控制，包含tim.h

// 函数声明
void Motor_Init(void);  // 电机初始化
void Motor_SetThrottle(uint8_t motor_id, float throttle);  // 设置单个电机推力（0-100%）
void Stabilize_Drone(float throttle, float dt);  // 维稳函数

#endif /* MOTOR_H_ */

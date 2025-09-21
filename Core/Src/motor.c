#include "motor.h"
#include "pid.h"
#include "MPU_6050.h"
#include "math.h"

// 电机初始化（假设使用TIM1的4个通道）
void Motor_Init(void) {
    // 初始化PWM定时器（在main中调用MX_TIM1_Init()后）
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 电机1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // 电机2
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // 电机3
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // 电机4
}

// 设置电机推力（throttle: 0-100%，映射到PWM占空比）
// 暂用throttle控制整体推力，PID输出用于微调，实现悬停效果
// 后续应当是修改为遥控器摇杆位置，对应不同期望姿态，再通过PID控制电机推力，实现姿态控制

void Motor_SetThrottle(uint8_t motor_id, float throttle) {
    uint32_t pwm_value = (uint32_t)(throttle * 1000.0f);  // PWM周期为1000（0-1000对应0-100%）

    switch (motor_id) {
        case 1: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value); break;
        case 2: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value); break;
        case 3: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_value); break;
        case 4: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_value); break;
        default: break;
    }
}

// 维稳函数（四轴无人机电机控制）
void Stabilize_Drone(float throttle, float dt) {
    // 计算误差
    float error_roll = target_roll - roll;
    float error_pitch = target_pitch - pitch;
    float error_yaw = target_yaw - yaw;

    // PID输出
    float pid_out_roll = PID_Update(&pid_roll, error_roll, dt);
    float pid_out_pitch = PID_Update(&pid_pitch, error_pitch, dt);
    float pid_out_yaw = PID_Update(&pid_yaw, error_yaw, dt);

    // 电机推力计算（假设电机1-4，范围0-100%）
    float motor1 = throttle + pid_out_pitch - pid_out_roll - pid_out_yaw;  // 前右
    float motor2 = throttle - pid_out_pitch - pid_out_roll + pid_out_yaw;  // 前左
    float motor3 = throttle + pid_out_pitch + pid_out_roll + pid_out_yaw;  // 后右
    float motor4 = throttle - pid_out_pitch + pid_out_roll - pid_out_yaw;  // 后左

    // 限制范围（0-100%）
    motor1 = fmaxf(0.0f, fminf(100.0f, motor1));
    motor2 = fmaxf(0.0f, fminf(100.0f, motor2));
    motor3 = fmaxf(0.0f, fminf(100.0f, motor3));
    motor4 = fmaxf(0.0f, fminf(100.0f, motor4));

    // 设置电机
    Motor_SetThrottle(1, motor1);
    Motor_SetThrottle(2, motor2);
    Motor_SetThrottle(3, motor3);
    Motor_SetThrottle(4, motor4);
}


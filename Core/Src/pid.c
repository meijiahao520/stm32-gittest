#include "pid.h"
#include "math.h"

PID_Controller pid_roll = {2.0f, 0.1f, 0.5f, 0.0f, 0.0f};  // PID参数，可调整（Kp, Ki, Kd，integral，prev_error）
PID_Controller pid_pitch = {2.0f, 0.1f, 0.5f, 0.0f, 0.0f};
PID_Controller pid_yaw = {1.0f, 0.05f, 0.2f, 0.0f, 0.0f};
float target_roll = 0.0f, target_pitch = 0.0f, target_yaw = 0.0f; // 目标姿态（水平稳定）


// PID更新函数 
float PID_Update(PID_Controller *pid, float error, float dt) {
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}


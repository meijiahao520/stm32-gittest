#ifndef PID_H_  /* Include guard */
#define PID_H_

#include "stdint.h"

// PID结构体
typedef struct {
    float Kp, Ki, Kd;  // PID参数
    float integral;    // 积分项
    float prev_error;  // 上次误差
} PID_Controller;

// 全局变量声明
extern PID_Controller pid_roll, pid_pitch, pid_yaw;
extern float target_roll, target_pitch, target_yaw;  // 目标姿态（度）

// 新增函数
float PID_Update(PID_Controller *pid, float error, float dt);

#endif  /* PID_H_ */



#ifndef PID_H_  /* Include guard */
#define PID_H_

#include "stdint.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float prev_derivative;
    float output;
} PIDController;

void PID_Init(PIDController *pid, float Kp, float Ki, float Kd);

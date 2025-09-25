#ifndef __PS2_H
#define __PS2_H

#include "stm32f1xx_hal.h"

// PS2状态结构体
typedef struct {
    uint8_t button_up;
    uint8_t button_down;
    uint8_t button_left;
    uint8_t button_right;
    uint8_t button_triangle;
    uint8_t button_circle;
    uint8_t button_cross;
    uint8_t button_square;
    uint8_t button_l1;
    uint8_t button_r1;
    uint8_t button_l2;
    uint8_t button_r2;
    uint8_t button_select;
    uint8_t button_start;
    uint16_t joystick_lx;
    uint16_t joystick_ly;
    uint16_t joystick_rx;
    uint16_t joystick_ry;
} PS2_Status;

// 函数声明
void PS2_Init(void);
uint8_t PS2_ReadData(uint8_t *data);
void PS2_ParseButtons(uint8_t *data, PS2_Status *status);

#endif
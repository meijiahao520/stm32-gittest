#include "ps2.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi1;  // 使用SPI1，假设在main.c中定义

// PS2手柄初始化
void PS2_Init(void) {
    // 初始化SPI（在main.c中调用HAL_SPI_Init）
    // 设置CS引脚为输出
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_4;  // CS引脚，调整为PA4
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // 默认高电平
}

// 读取PS2手柄数据
uint8_t PS2_ReadData(uint8_t *data) {
    uint8_t cmd[5] = {0x01, 0x42, 0x00, 0x00, 0x00};  // PS2读取命令
    uint8_t recv[5];
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // CS低
    HAL_SPI_TransmitReceive(&hspi1, cmd, recv, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // CS高
    
    for (int i = 0; i < 5; i++) {
        data[i] = recv[i];
    }
    return 1;  // 成功
}

// 解析按钮状态
void PS2_ParseButtons(uint8_t *data, PS2_Status *status) {
    if (data[1] != 0x73) return;  // 检查响应
    status->button_up = !(data[3] & 0x10);
    status->button_down = !(data[3] & 0x40);
    status->button_left = !(data[3] & 0x80);
    status->button_right = !(data[3] & 0x20);
    status->button_triangle = !(data[4] & 0x10);
    status->button_circle = !(data[4] & 0x20);
    status->button_cross = !(data[4] & 0x40);
    status->button_square = !(data[4] & 0x80);
    status->button_l1 = !(data[4] & 0x04);
    status->button_r1 = !(data[4] & 0x08);
    status->button_l2 = !(data[4] & 0x01);
    status->button_r2 = !(data[4] & 0x02);
    status->button_select = !(data[3] & 0x01);
    status->button_start = !(data[3] & 0x08);
    status->joystick_lx = data[5];
    status->joystick_ly = data[6];
    status->joystick_rx = data[7];
    status->joystick_ry = data[8];
}
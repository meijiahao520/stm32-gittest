#include "nrf24L01.h"
#include "spi.h"
#include "main.h"

void delay_us(uint32_t udelay)    // 基于 HAL 的微秒级延时（HAL_Delay仅支持毫秒级延时,无法满足 PS2 通信中所需的微秒级精度）
{
  uint32_t startval,tickn,delays,wait;
 
  startval = SysTick->VAL;
  tickn = HAL_GetTick();
    //sysc = 72000;  // SystemCoreClock / (1000U / uwTickFreq)
    delays =udelay * 72; // 等效转换为 SysTick 计数
  if(delays > startval)
    {
            while(HAL_GetTick() == tickn)
        {
 
        }
      wait = 72000 + startval - delays;
      while(wait < SysTick->VAL)
        {
 
        }
    }
  else
    {
      wait = startval - delays;
      while(wait < SysTick->VAL && HAL_GetTick() == tickn)
        {
 
        }
    }
}


void NRF24L01_Init(void) {
    // 初始化NRF24L01
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
    delay_us(100);
    if (NRF24L01_Check() != 1) {  // 检查通信是否正常
        Error_Handler();
    }
}


// 封装：写单个字节并读单个字节
uint8_t  SPI_WriteByte(uint8_t data) {
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi2, &data, &rx_data, 1, 100);  // 发送并接收（NRF24L01 常需）
    return rx_data;  // 返回接收到的字节（如果不需要，可忽略）
}


// 封装：写多字节（用于发送数据包）
void SPI_WriteBytes(uint8_t *data, uint8_t length) {
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    for (uint8_t i = 0; i < length; i++) {
        SPI_WriteByte(data[i]);
    }
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}


// 封装：读多字节
void SPI_ReadBytes(uint8_t *data, uint8_t length) {
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    for (uint8_t i = 0; i < length; i++) {
        uint8_t tx_data = 0xFF;  // 发送空字节以读取数据
        data[i] = SPI_WriteByte(tx_data);  // 读取数据并存储到缓冲区
    }
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}


// 封装：写寄存器（命令 + 数据）
void SPI_WriteReg(uint8_t reg, uint8_t value) {
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);  // 拉低 CSN
    SPI_WriteByte(reg);  // 发送寄存器地址（写命令）
    SPI_WriteByte(value);  // 发送数据
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);  // 拉高 CSN
}


// 检查 NRF24L01 通信是否正常
// 检查nrf24L01是否通信正常，主要思想是，往TX_ADDR地址寄存器中写入了5个数，然后再读出来，对比是否一样，一样则认为通信成功：
unsigned char NRF24L01_Check(void) {
    uint8_t test_addr[5] = {0x12, 0x34, 0x56, 0x78, 0x9A};  // 测试地址
    uint8_t read_addr[5] = {0};  // 读取缓冲区

    // 写入测试地址到 TX_ADDR 寄存器
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    SPI_WriteByte(NRF24L01_CMD_W_REGISTER | NRF24L01_REG_TX_ADDR);  // 发送 后续要写TX_ADDR的命令
    SPI_WriteBytes(test_addr, 5);  // 写入 5 字节
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

    delay_us(10);  // 小延时确保写入完成

    // 读取 TX_ADDR 寄存器
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    SPI_WriteByte(NRF24L01_CMD_R_REGISTER | NRF24L01_REG_TX_ADDR);  // 发送 后续要读TX_ADDR的命令
    SPI_ReadBytes(read_addr, 5);  // 读取 5 字节
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

    // 对比写入和读取的数据
    for (uint8_t i = 0; i < 5; i++) {
        if (test_addr[i] != read_addr[i]) {
            return 0;  // 通信失败
        }
    }
    return 1;  // 通信成功
}


// 接收数据函数（从 NRF24L01 读取接收到的数据）
uint8_t NRF24L01_Receive(uint8_t *data, uint8_t length) {
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    SPI_WriteByte(NRF24L01_CMD_R_RX_PAYLOAD);  // 发送读接收负载命令
    SPI_ReadBytes(data, length);  // 读取数据
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
    return 1;  // 成功（可扩展错误检查）
}

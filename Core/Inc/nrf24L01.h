#ifndef __NRF24L01_H
#define __NRF24L01_H
#endif

#define NRF24L01_CMD_R_REGISTER 0x00
#define NRF24L01_CMD_W_REGISTER 0x20
#define NRF24L01_REG_TX_ADDR 0x10
#define NRF24L01_REG_RX_ADDR_P0 0x0A

void delay_us(uint32_t udelay)
void NRF24L01_Init(void);
void NRF24L01_Transmit(uint8_t* data, uint8_t length);
void NRF24L01_Receive(uint8_t* data, uint8_t length);

#endif /* __NRF24L01_H */
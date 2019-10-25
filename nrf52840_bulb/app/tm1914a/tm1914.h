#ifndef TM1914_H
#define TM1914_H 1

#define u8 unsigned char

#define DIN          NRF_GPIO_PIN_MAP(0,13)
#define FDIN         NRF_GPIO_PIN_MAP(0,14)
void TM1914_Init(void);
void TM1914_SendByte(u8 b);
void TM1914_SetData(u8 r,u8 g,u8 b,int num);
void TM1914_SendFrame(void);
void TM1914_Test(void);
#endif
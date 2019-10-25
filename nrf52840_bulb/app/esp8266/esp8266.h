
#include <string.h> 
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#define	FALSE					0
#define	TRUE					1

#define Buf_Max 50


#define ESP8266_RST_H   nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0,13))
#define ESP8266_RST_L   nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0,13))

/****************************************************************
ATָ�
*****************************************************************/	



void esp8266_init(void);
void esp8266_test(void);
void esp8266_asmod(void);
void USART3_Init(void);
void CLR_Buf(void);
void USART3_SendByte(uint8_t byte);
bool Hand( char *a);


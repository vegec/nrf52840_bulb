#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"
#include "esp8266.h"
#include "w5500.h"
#include "tm1914.h"
#define RXBUF_LEN  3              //应用程序UART接收缓存字节数
#define UART_TX_BUF_SIZE 256       //串口发送缓存大小（字节数）
#define UART_RX_BUF_SIZE 256       //串口接收缓存大小（字节数）

//SPI发送缓存数组，使用EasyDMA时一定要定义为static类型
static uint8_t    my_tx_buf[4096];  
//SPI接收缓存数组，使用EasyDMA时一定要定义为static类型
static uint8_t    my_rx_buf[4096];  
extern  char Rx232buffer[Buf_Max]; 
void test(void);

/***************************************************************************
* 描  述 : 设置GPIO高电平时的输出电压为3.3V 
* 入  参 : 无 
* 返回值 : 无
**************************************************************************/
static void gpio_output_voltage_setup_3v3(void)
{
    //读取UICR_REGOUT0寄存器，判断当前GPIO输出电压设置的是不是3.3V，如果不是，设置成3.3V
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) !=
        (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        //复位更新UICR寄存器
        NVIC_SystemReset();
    }
}
int main(void)
{
		uint8_t RxCnt = 0;             //UART接收字节数
	  uint8_t UartRxBuf[RXBUF_LEN];  //UART接收缓存

	//设置GPIO输出电压为3.3V
  gpio_output_voltage_setup_3v3();
  bsp_board_init(BSP_INIT_LEDS);
  esp8266_init();
	SPI_W5500_Init();              //初始化SPI和外部中断
	Load_Net_Parameters();		    //装载网络参数	
	W5500_Hardware_Reset();		    //硬件复位W5500
	W5500_Initialization();		    //W5500初始货配置
	TM1914_Init();
	
	while(true)
	{
	test();
	}
}
void test()
{
	TM1914_SendFrame();
	W5500_Socket_Set();           //W5500端口初始化配置
	if(Hand("LED"))                      //  收到打开LED1的指令
	{
		nrf_gpio_pin_toggle(LED_1);           //点亮指示灯D1
		for(int i=0;i<50;i++)
			if(Rx232buffer[i]=='L'&&Rx232buffer[i+1]=='E'&&Rx232buffer[i+2]=='D')
			{
				TM1914_SetData(Rx232buffer[i+4],Rx232buffer[i+5],Rx232buffer[i+6],Rx232buffer[i+3]-48);
			}
		CLR_Buf();			
	}  

			
		
    if(W5500_Interrupt)         //处理W5500中断
	  {
			W5500_Interrupt_Process();//W5500中断处理程序框架
		}

		if((S0_Data & S_RECEIVE) == S_RECEIVE)//如果Socket0接收到数据
		{
			S0_Data&=~S_RECEIVE;
			Read_SOCK_Data_Buffer(0, Rx_Buffer);
            if(Rx_Buffer[0]=='2')		
               nrf_gpio_pin_toggle(LED_2);				
				
		}
}

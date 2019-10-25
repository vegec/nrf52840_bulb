#include "esp8266.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define UART_TX_BUF_SIZE 256       //串口发送缓存大小（字节数）
#define UART_RX_BUF_SIZE 256       //串口接收缓存大小（字节数）

#define RXBUF_LEN  3              //应用程序UART接收缓存字节数

volatile uint8_t Count=0;                                                      // 接收数组累加计数       
char Rx232buffer[Buf_Max];                                                     // 接收数组


void uart_error_handle(app_uart_evt_t * p_event)
{
    uint8_t cr;
	
	  //通讯错误事件
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    //FIFO错误事件
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
		//串口接收事件
    else if (p_event->evt_type == APP_UART_DATA_READY)
    {
			app_uart_get(&cr);
			Rx232buffer[Count]=cr;          //通过外设USART3接收数据
			Count++;                                                //不断累加接收数据                          
	  if(Count>Buf_Max)                                       //接收数大于定义接收数组最大个数时，从头开始放置数据
		{
			 Count = 0;
		} 
		}
  
		//串口发送完成事件
    else if (p_event->evt_type == APP_UART_TX_EMPTY)
    {
        //翻转指示灯D2状态，指示串口发送完成事件
			 // nrf_gpio_pin_toggle(LED_2);
    }
}
/**************************************************************************************
 * 描  述 : 初始化ESP8266模块用到的引脚
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
void esp8266_init(void)
{
	uint32_t err_code;
	
	//定义串口通讯参数配置结构体并初始化
  const app_uart_comm_params_t comm_params =
  {
    RX_PIN_NUMBER,//定义uart接收引脚
    TX_PIN_NUMBER,//定义uart发送引脚
    RTS_PIN_NUMBER,//定义uart RTS引脚，流控关闭后虽然定义了RTS和CTS引脚，但是驱动程序会忽略，不会配置这两个引脚，两个引脚仍可作为IO使用
    CTS_PIN_NUMBER,//定义uart CTS引脚
    APP_UART_FLOW_CONTROL_DISABLED,//关闭uart硬件流控
    false,//禁止奇偶检验
    NRF_UART_BAUDRATE_115200//uart波特率设置为115200bps
  };
  //初始化串口，注册串口事件回调函数
  APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

  APP_ERROR_CHECK(err_code);
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 5));
	nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0, 5));
	nrf_delay_ms(500);
	esp8266_test();
  esp8266_asmod();   
}

void esp8266_test(void)
{
	while(!Hand("OK"))                        //判断是否设置成功，如不成功，延时后再次发送
	{		
		  printf("AT\r\n");                     //发送联机指令"AT"
		  nrf_delay_ms(500);                     //该延时不可少
	}
  CLR_Buf();                                //清空接收数组数据
  nrf_gpio_pin_clear(LED_3);                            //完成ESP8266模块联机指令后，点亮指示灯D3
}

void esp8266_asmod(void)
{
	 //向ESP8266模块发送设置ESP8266工作模式的命令，返回"OK"或者"no change"
	while(!(Hand("OK") | Hand("no change")))   //判断是否设置成功，如不成功，延时后再次发送
	{		
		  printf("AT+CWMODE=3\r\n");            //发送设置ESP8266工作模式为AP+STATION模式
		  nrf_delay_ms(500);                     //该延时不可少
	}
  //返回"OK"的话
//  if(Hand("OK")) 
//  {		
//		  printf("AT+RST\r\n");                 //启动模块，软件复位
//		  nrf_delay_ms(500);                     //该延时不可少
//	 }
  CLR_Buf();                                //清空接收数组数据
  //向ESP8266模块发送设置ESP8266工作在多路连接模式的命令
	while(!(Hand("OK")))                       //判断是否设置成功，如不成功，延时后再次发送
	{		
		  printf("AT+CIPMUX=1\r\n");            //设置ESP8266工作在多路连接模式
		  nrf_delay_ms(500);                     //该延时不可少
	 }
  CLR_Buf();                                //清空接收数组数据
  //向ESP8266模块发送建立TCP服务器且开放端口为5000的命令
	while(!(Hand("OK")))                       //判断是否设置成功，如不成功，延时后再次发送
	{		
		  printf("AT+CIPSERVER=1,5000\r\n");    //建立TCP服务器且开放端口为5000
		  nrf_delay_ms(500);                     //该延时不可少
	 }
  CLR_Buf();                                //清空接收数组数据
  //向ESP8266模块发送获取本地IP地址的命令
	while(!(Hand("OK")))                       //判断是否设置成功，如不成功，延时后再次发送
	{		
		  printf("AT+CIFSR\r\n");               //获取本地IP地址
		  nrf_delay_ms(500);                     //该延时不可少
	 }
  CLR_Buf();                                //清空接收数组数据
}



/**************************************************************************************
 * 描  述 : 握手成功与否函数
 * 入  参 : char *a待对比字符串
 * 返回值 : 无
 **************************************************************************************/
bool Hand( char *a)
{ 
  if(strstr(Rx232buffer,a)!=NULL)    //判断指针a中的字符串是否是Rx232buffer数组中字符串的子串
	   return true;
	else
		 return false;
}

/**************************************************************************************
 * 描  述 : 清空接收数组数据
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
void CLR_Buf(void)
{
	uint8_t k;
  for(k=0;k<Buf_Max;k++)      
  {
			Rx232buffer[k] = 0;
	}      
  Count = 0;                                         //接收数组累加计数清零，即下次接收从头开始
}



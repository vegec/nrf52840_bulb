/**************（C）COPYRIGHT 2019 射频与集成电路与系统研究中心*******************
* 文件名   : tm1914.c
* 描述     : 初始化并驱动tm1914
* 实验平台 : STM32F103ZET6
* 版本     : V0.1
* 嵌入系统 : 无
* 备注     : 
*******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "tm1914.h"
#define BulbNum  5
u8 BulbData[BulbNum*3]={0};
/**************（C）COPYRIGHT 2019 射频与集成电路与系统研究中心*******************
* 函数名   : TM1914_Init
* 功能说明 : 初始化向TM1914传输数据的引脚
* 参数说明 : 无
* 函数返回 : 无
* 备注     : 此处设定引脚为PB8、PB9，通过灯光的明灭测试数据的传输
* 作者     :杜镇枭
* 修改时间 :2019.9.18
*******************************************************************************/
void TM1914_Init(void)
{
	nrf_gpio_cfg_output(DIN);
	nrf_gpio_cfg_output(FDIN);
}
/**************（C）COPYRIGHT 2019 射频与集成电路与系统研究中心*******************
* 函数名   : TM1914_SendByte
* 功能说明 : 发送一个字节至TM1914
* 参数说明 : b - 准备发送的字节
* 函数返回 : 无
* 备注     : 低电平时间为360±50ns时，代表数据0
			   低电平时间为650-1000ns时，代表数据1
			   一个bit的周期为1.25μs（频率 800KHz）至 2.5μs（频率 400KHz）范围内
			   该代码的延时时间需要自行测试
* 作者     :杜镇枭
* 修改时间 :2019.9.18
*******************************************************************************/
void TM1914_SendByte(u8 b)//发送一个字节
{//延时是根据主频48M计算,输出管脚可以自己定义
 int i;
	for(i=0;i<8;i++)
	{
		if(b&0x80)
		{
 			nrf_gpio_pin_clear(DIN);	//0
			nrf_gpio_pin_clear(FDIN);
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();

///
 			nrf_gpio_pin_set(DIN);	//1
			nrf_gpio_pin_set(FDIN);
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();

		}
		else
		{
     	nrf_gpio_pin_clear(DIN);	//0
			nrf_gpio_pin_clear(FDIN);
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
    __nop();
		__nop();
     	nrf_gpio_pin_set(DIN);	//1
			nrf_gpio_pin_set(FDIN);
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		}
		b<<=1;
  }
}
/**************（C）COPYRIGHT 2019 射频与集成电路与系统研究中心*******************
* 函数名   : TM1914_SetData
* 功能说明 : 在发送数据包里设定一个指定灯泡的数据
* 参数说明 : r - 指定灯泡的r通道的值
						 g - 指定灯泡的g通道的值
						 b - 指定灯泡的b通道的值
						 num - 灯泡编号
* 函数返回 : 无
* 备注     : 
* 作者     :杜镇枭
* 修改时间 :2019.9.18
*******************************************************************************/
void TM1914_SetData(u8 r,u8 g,u8 b,int num)
{
	  u8 *p;
		p=&BulbData[(num-1)*3];
	   *p++=r;
	   *p++=g;
	   *p++=b;
}
/**************（C）COPYRIGHT 2019 射频与集成电路与系统研究中心*******************
* 函数名   : TM1914_SendFrame
* 功能说明 : 向TM1914发送完整的一帧数据
* 参数说明 : 无
* 函数返回 : 无
* 备注     : 
* 作者     :杜镇枭
* 修改时间 :2019.9.18
*******************************************************************************/
void TM1914_SendFrame(void)
{
	 u8 *p;
	 int i;	
	 TM1914_SendByte(0xff);
	 TM1914_SendByte(0xff);
	 TM1914_SendByte(0xff);
	 TM1914_SendByte(0x00);
	 TM1914_SendByte(0x00);
	 TM1914_SendByte(0x00);	
	 p=(void*)BulbData;
	 for(i=0;i<BulbNum*3;i++)
	 {
	  TM1914_SendByte(*p++);
	 }	

}
void TM1914_Test(void)
{

TM1914_SendByte(0xAA);
}
#include "ws2812.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "data_pool.h"
#include "drive_tim.h"
/*
@brief 用于驱动led灯条
@param 使用时需要将spi1配置为5mhz，使用主机发送即可


*/
 
//灯条显存SPI数据缓存
uint8_t gWs2812bDat_SPI[WS2812B_AMOUNT * 24] = {0};  

//灯条显存
tWs2812bCache_TypeDef gWs2812bDat[WS2812B_AMOUNT] =
{
//R    G      B
0X10, 0X10, 0X10,	//0
0X10, 0X10, 0X10,	//1
0X10, 0X10, 0X10,	//2
0X10, 0X10, 0X10,	//3
0X10, 0X10, 0X10,	//4
0X10, 0X10, 0X10,	//5
0X10, 0X10, 0X10,	//6
0X10, 0X10, 0X10,	//7
0X10, 0X10, 0X10,	//8
0X10, 0X10, 0X10,	//9
0X10, 0X10, 0X10,	//10
0X10, 0X10, 0X10,	//11
0X10, 0X10, 0X10,	//12
0X10, 0X10, 0X10,	//13
0X10, 0X10, 0X10,	//14
0X10, 0X10, 0X10,	//15
0X10, 0X10, 0X10,	//16
0X10, 0X10, 0X10,	//17
0X10, 0X10, 0X10,	
0X10, 0X10, 0X10,	
0X10, 0X10, 0X10,	
0X10, 0X10, 0X10,	
0X10, 0X10, 0X10,	
0X10, 0X10, 0X10,	
0X10, 0X10, 0X10,	
0X10, 0X10, 0X10,	
0X10, 0X10, 0X10,	
0X10, 0X10, 0X10,	
0X10, 0X10, 0X10,	
0X10, 0X10, 0X10
};

void WS2812b_Set(uint16_t Ws2b812b_NUM, uint8_t r,uint8_t g,uint8_t b)
{
	uint8_t *pR = &gWs2812bDat_SPI[(Ws2b812b_NUM) * 24 + 8];
	uint8_t *pG = &gWs2812bDat_SPI[(Ws2b812b_NUM) * 24];
	uint8_t *pB = &gWs2812bDat_SPI[(Ws2b812b_NUM) * 24 + 16];
	
	for(uint8_t i = 0; i <  8; i++)
	{
		if(g & 0x80) *pG = CODE_1;           
		else *pG = CODE_0;
		           
		if(r & 0x80) *pR = CODE_1;        
		else *pR = CODE_0;
		           
		if(b & 0x80) *pB = CODE_1;    
		else *pB = CODE_0;
		
		r <<= 1;
		g <<= 1;
		b <<= 1;
		pR++;
		pG++;
		pB++;
	}
}

#define RESET_BYTES 100



#define TIME_INTERVAL 80000
//利用 颜色来判断重定位是否完成
void WS2812B_Task(void *argument)
{	
	static Ws2812b_SIGNAL_T signal;
	
	for (;;)
	{
		if(xQueueReceive(Send_WS2812_Port, &signal, 1) == pdPASS)
		{
			static uint8_t flag = 0;
			static uint32_t start_time = 0;
			uint32_t current_time = Get_SystemTimer();
			
			
			if (flag % 2 == 0 && signal == SIGNAL_FAIL)
			{
				flag = 1;
				LED_FAIL();
				start_time = current_time;
			}
		
			
			
			switch (flag)
			{
				case 0:
					
					if (signal == SIGNAL_NORMAL)
					{
						LED_NORMAL();
					}
					else if (signal == SIGNAL_WAIT)
					{
						LED_WAIT();
					}
					else if (signal == SIGNAL_SUCCESS)
					{
						flag = 2;
						LED_SUCCESS();
						start_time = current_time;
						
					}
					else if (signal == SIGNAL_FAIL)
					{
						flag = 1;
						LED_FAIL();
						start_time = current_time;
						
					}
					break;
					
				case 2:
					
					if (current_time - start_time > TIME_INTERVAL)
					{
						flag = 4;
						LED_OFF();
						start_time = current_time;
					}
					break;
				
				case 4:
					if (current_time - start_time > TIME_INTERVAL)
					{
						flag = 6;
						LED_SUCCESS();
						start_time = current_time;
					}
					break;
				
				case 6:
							
					if (current_time - start_time > TIME_INTERVAL)
					{
						flag = 0;
						LED_OFF();
						start_time = current_time;
					}
					break;
				
				
				case 1:
					if (current_time - start_time > TIME_INTERVAL)
					{
						flag = 3;
						LED_OFF();
						start_time = current_time;
					}
					break;
				
				case 3:
					if (current_time - start_time > TIME_INTERVAL)
					{
						flag = 5;
						LED_FAIL();
						start_time = current_time;
					}
					break;
				
				case 5:
					if (current_time - start_time > TIME_INTERVAL)
					{
						flag = 0;
						LED_OFF();
						start_time = current_time;
					}
					break;
				
				default:
					flag = 0;
					break;
			}
			
			
	
			
			
		
		
			
			WS2812b_Send();
		}
		osDelay(1);
	}
}



void WS2812b_Send(void)
{
	uint8_t reset_buf[RESET_BYTES] = {0};
	
	//将gWs2812bDat数据解析成SPI数据
	for(uint8_t iLED = 0; iLED < WS2812B_AMOUNT; iLED++)
	{
		WS2812b_Set(iLED, gWs2812bDat[iLED].R, gWs2812bDat[iLED].G, gWs2812bDat[iLED].B);
	}
	
	//总线输出数据
	HAL_SPI_Transmit_DMA(&hspi1, gWs2812bDat_SPI, sizeof(gWs2812bDat_SPI));
	
	//使总线输出低电平
	HAL_SPI_Transmit_DMA(&hspi1, reset_buf, RESET_BYTES);
	//帧信号：一个大于50us的低电平
}


















// 设置全部灯为指定颜色
void WS2812B_SetAllColor(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint8_t i = 0; i < WS2812B_AMOUNT; i++)
	{
        gWs2812bDat[i].R = r;
        gWs2812bDat[i].G = g;
        gWs2812bDat[i].B = b;
    }
}




void LED_FAIL(void)
{
	WS2812B_SetAllColor(0x08,0x00,0x00);//red
}

void LED_WAIT(void)
{
	WS2812B_SetAllColor(0x03, 0x03, 0x03);//
}

void LED_SUCCESS(void)
{
	WS2812B_SetAllColor(0x00, 0x08, 0x00);//green
}

void LED_NORMAL(void)
{
	WS2812B_SetAllColor(0x00, 0x00, 0x08);//blue	
}

void LED_OFF(void)
{
	WS2812B_SetAllColor(0x00, 0x00, 0x00);//	
}


void WS2812B_Send_FAIL(void)
{
	Ws2812b_SIGNAL_T Ws2812b_signal = SIGNAL_FAIL;
	xQueueSend(Send_WS2812_Port, &Ws2812b_signal, 0);
}


void WS2812B_Send_SUCCESS(void)
{
	Ws2812b_SIGNAL_T Ws2812b_signal = SIGNAL_SUCCESS;
	xQueueSend(Send_WS2812_Port, &Ws2812b_signal, 0);
}



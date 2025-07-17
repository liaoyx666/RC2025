#include "ws2812.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
 
//灯条显存SPI数据缓存
uint8_t gWs2812bDat_SPI[WS2812B_AMOUNT * 24] = {0};  

//灯条显存
tWs2812bCache_TypeDef gWs2812bDat[WS2812B_AMOUNT] =
{
//R    G      B
0X20, 0X20, 0X20,	//0
0X20, 0X20, 0X20,	//1
0X20, 0X20, 0X20,	//2
0X20, 0X20, 0X20,	//3
0X20, 0X20, 0X20,	//4
0X20, 0X20, 0X20,	//5
0X20, 0X20, 0X20,	//6
0X20, 0X20, 0X20,	//7
0X20, 0X20, 0X20,	//8
0X20, 0X20, 0X20,	//9
0X20, 0X20, 0X20,	//10
0X20, 0X20, 0X20,	//11
0X20, 0X20, 0X20,	//12
0X20, 0X20, 0X20,	//13
0X20, 0X20, 0X20,	//14
0X20, 0X20, 0X20,	//15
0X20, 0X20, 0X20,	//16
0X20, 0X20, 0X20,	//17
0X20, 0X20, 0X20,	
0X20, 0X20, 0X20,	
0X20, 0X20, 0X20,	
0X20, 0X20, 0X20,	
0X20, 0X20, 0X20,	
0X20, 0X20, 0X20,	
0X20, 0X20, 0X20,	
0X20, 0X20, 0X20,	
0X20, 0X20, 0X20,	
0X20, 0X20, 0X20,	
0X20, 0X20, 0X20,	
0X20, 0X20, 0X20
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

#define RESET_BYTES 64 

void WS2812B_Task(void *argument)
{
	uint8_t reset_buf[RESET_BYTES] = {0};
	
	for (;;)
	{
		//将gWs2812bDat数据解析成SPI数据
		for(uint8_t iLED = 0; iLED < WS2812B_AMOUNT; iLED++)
		{
			WS2812b_Set(iLED, gWs2812bDat[iLED].R, gWs2812bDat[iLED].G, gWs2812bDat[iLED].B);
		}
		
		//总线输出数据
		HAL_SPI_Transmit(&hspi1, gWs2812bDat_SPI, sizeof(gWs2812bDat_SPI), 100);
		
		//使总线输出低电平
		HAL_SPI_Transmit(&hspi1, reset_buf, RESET_BYTES, 100);
		//帧信号：一个大于50us的低电平
		
		osDelay(10);
	}
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

void LED_WARNING(void)
{
	WS2812B_SetAllColor(0x20,0x00,0x00);//red
}

void LED_WAITING(void)
{
	WS2812B_SetAllColor(0x00, 0x00, 0x20); //blue	
}

void LED_SUCCESS(void)
{
	WS2812B_SetAllColor(0x00, 0x10, 0x00);//green
}
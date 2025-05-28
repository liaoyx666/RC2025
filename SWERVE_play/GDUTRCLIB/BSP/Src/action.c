#include "action.h"
#include <math.h>

RawPos RawPosData = {0};
RealPos RealPosData = {0};

union
{
	uint8_t data[20];
	float ActVal[5];
} posture;

uint32_t Position_UART3_RxCallback(uint8_t *buf, uint16_t len)
{
	uint8_t count = 0;
	uint8_t i = 0;
	uint8_t CRC_check[2];
	
	
	
	uint8_t break_flag = 1;
	while(i < len && break_flag == 1)
	{
		switch (count)
		{
			case 0:
			{
				if (buf[i] == 0xfc)   //接收包头1
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			case 1:
			{
				if (buf[i] == 0xfb) //接收包头2
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			
			case 2:
			{
				if (buf[i] == 0x01) 
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			
			case 3:
			{
				if (buf[i] == 0x14) 
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			
			
			case 4:
			{
				uint8_t j;
				
				
				if (i > len - 28)
				{
					break_flag = 0;
				}
				
				for(j = 0; j < 20; j++)
				{
					posture.data[j] = buf[i];
					i++;
				}
				count++;
				break;
			}
			
			
			case 5:
			{
				uint8_t j;
				
				for(j = 0; j < 2; j++)
				{
					CRC_check[j] = buf[i];
					i++;
				}
				count++;
				break;
			}
			
			case 6:
			{
				if (buf[i] == 0xfd)  //接收包尾1
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			case 7:
			{
				if (buf[i] == 0xfe)  //接收包尾2
				{	
					//在接收包尾2后才开始启动回调
					Update_RawPosition(posture.ActVal);
				}
				count = 0;
				
				break_flag = 0;
				
				break;
			}
			
			default:
			{
				count = 0;
				break;
			}
		}
		
	}
	return 0;
}



void Update_RawPosition(float value[5])
{
//	//赋值
//	RawPosData.LAST_Pos_X = RawPosData.Pos_X;
//	RawPosData.LAST_Pos_Y = RawPosData.Pos_Y;

	//处理数据
	RawPosData.Pos_X = value[0] / 1000.f; 
	RawPosData.Pos_Y = value[1] / 1000.f; 
	RawPosData.angle_Z = value[2];
	RawPosData.Speed_X = value[3];
	RawPosData.Speed_Y = value[4];

//   //差分运算
//	RawPosData.DELTA_Pos_X = RawPosData.Pos_X - RawPosData.LAST_Pos_X;
//	RawPosData.DELTA_Pos_Y = RawPosData.Pos_Y - RawPosData.LAST_Pos_Y;

  //世界坐标
	RealPosData.world_yaw = RawPosData.angle_Z;
  
	//加入安装误差
	RawPosData.REAL_X += (RawPosData.DELTA_Pos_X);
	RawPosData.REAL_Y += (RawPosData.DELTA_Pos_Y);
	
  //解算安装误差
	RealPosData.world_x = RawPosData.REAL_X + INSTALL_ERROR_X * sinf(RealPosData.world_yaw * PI / 180.f);
	RealPosData.world_y = RawPosData.REAL_Y + INSTALL_ERROR_Y * cosf(RealPosData.world_yaw * PI / 180.f);
}








void POS_Change(float X, float Y)
{
	  //定义接收缓冲区
    uint8_t txBuffer[10];
	//使用联合体以便将浮点型数据转换为字节并发送
    union
	{
        float f;
        uint8_t bytes[4];
    } floatUnion;
    // 起始标志
    txBuffer[0] = 0x01;  
    //从输入的X中取出四个字节
    floatUnion.f = X;
    txBuffer[1] = floatUnion.bytes[0];
    txBuffer[2] = floatUnion.bytes[1];
    txBuffer[3] = floatUnion.bytes[2];
    txBuffer[4] = floatUnion.bytes[3];
    //同X
    floatUnion.f = Y;
    txBuffer[5] = floatUnion.bytes[0];
    txBuffer[6] = floatUnion.bytes[1];
    txBuffer[7] = floatUnion.bytes[2];
    txBuffer[8] = floatUnion.bytes[3];
    //字节长度
    txBuffer[9] = 0x02; 
    //逐一发送，这里使用的是阻塞式，因为校准的时候并不会移动，无需使用DMA
    HAL_UART_Transmit(&huart3, txBuffer, 10, HAL_MAX_DELAY);
}






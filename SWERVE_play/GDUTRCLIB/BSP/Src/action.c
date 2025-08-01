#include "action.h"
#include <math.h>


RawPos RawPosData = {0, .sin_yaw_offset = 0, .cos_yaw_offset = 1};
RealPos RealPosData = {0};


float GetYawError(float target, float current)
{
	float error = target - current;
	// 将误差归一化到-180度到180度之间
	while (error > 180.0f)
	{
		error -= 360.0f;
	}
	
	while (error < -180.0f)
	{
		error += 360.0f;
	}
	return error;
}
/*
  @使用方法：
  使用串口3进行通讯，注意不是232！！！！
  
  action数据存放在ActVal[5]中，如果要调用请外部申明
  从第一个数据到第五个数据分别是POS_X,POS_Y,YAW,SPEED_X,SPEED_Y
  在.h文件中需要填入安装误差
  转换到车中心的坐标存放在ROBOT_REAL_POS_DATA[]中，如果要调用请外部申明
  更新XY坐标时需要调用POS_UPDATE();直接输入两个浮点型数据即可。
  
  此文件用于position！！！！！！！！！
*/




union
{
	uint8_t data[20];
	float ActVal[5];
} posture;

uint32_t Position_UART3_RxCallback(uint8_t *buf, uint16_t len)
{
	uint8_t count = 0;
	uint8_t i = 0;
	uint8_t CRC_check[2];//CRC校验位，此文件未启用
	
	
	
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
			
			
			case 2://接收帧ID和数据长度
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
			
			
			
			case 4://开始接收数据
			{
				uint8_t j;
				
				
				if (i > len - 24)
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
			
			//接收CRC校验码
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
	//处理数据
	RawPosData.Pos_X = value[0] / 1000.f; 
	RawPosData.Pos_Y = value[1] / 1000.f; 
	RawPosData.angle_Z = value[2];
	RawPosData.Speed_X = value[3];
	RawPosData.Speed_Y = value[4];

	RealPosData.raw_x = RawPosData.Pos_X;
	RealPosData.raw_y = RawPosData.Pos_Y;
  
	RealPosData.world_yaw = GetYawError(RawPosData.angle_Z - RawPosData.yaw_offset, 0);
	RealPosData.raw_yaw = RawPosData.angle_Z;

	
	if ((RawPosData.x_offset != 0) || (RawPosData.y_offset != 0) || (RawPosData.yaw_offset != 0))
	{
		RealPosData.world_x =   RawPosData.Pos_X * (float)RawPosData.cos_yaw_offset + RawPosData.Pos_Y * (float)RawPosData.sin_yaw_offset;
		RealPosData.world_y =  -RawPosData.Pos_X * (float)RawPosData.sin_yaw_offset + RawPosData.Pos_Y * (float)RawPosData.cos_yaw_offset;
		
		RealPosData.world_x -= RawPosData.x_offset;
		RealPosData.world_y -= RawPosData.y_offset;
	}
	else
	{
		RealPosData.world_x = RawPosData.Pos_X;
		RealPosData.world_y = RawPosData.Pos_Y;
	}
}



#define FRAME_HEAD_POSITION_0 0xfc  //包头
#define FRAME_HEAD_POSITION_1 0xfb

#define FRAME_TAIL_POSITION_0 0xfd  //包尾
#define FRAME_TAIL_POSITION_1 0xfe



//1:更新X，Y
//2:更新X，Y，Yaw
//3:更新X，Y，重置陀螺仪

void Reposition_SendData(uint8_t id, float X, float Y, float Yaw)
{
	uint8_t txBuffer[20] = {0};

	union
	{
        float f;
        uint8_t bytes[4];
    } floatUnion;
	
	switch(id)
	{
	case 1:
		//数据长度
		txBuffer[3] = 0x08;
		break;
	
	case 2:
		//数据长度
		txBuffer[3] = 0x0c;
		break;
	
	case 3:
		//数据长度
		txBuffer[3] = 0x08;
		break;
	
	default:
		return;
	}
	
	//包头
	txBuffer[0] = FRAME_HEAD_POSITION_0;
	txBuffer[1] = FRAME_HEAD_POSITION_1;
    txBuffer[2] = id;
	
	//3
	
	//数据
	floatUnion.f = X;
	txBuffer[4] = floatUnion.bytes[0];
    txBuffer[5] = floatUnion.bytes[1];
    txBuffer[6] = floatUnion.bytes[2];
    txBuffer[7] = floatUnion.bytes[3];

    floatUnion.f = Y;
    txBuffer[8] = floatUnion.bytes[0];
    txBuffer[9] = floatUnion.bytes[1];
    txBuffer[10] = floatUnion.bytes[2];
    txBuffer[11] = floatUnion.bytes[3];

	switch(id)
	{
	case 1:
		//CRC
		txBuffer[12] = 0;
		txBuffer[13] = 0;
		//包尾
		txBuffer[14] = FRAME_TAIL_POSITION_0;
		txBuffer[15] = FRAME_TAIL_POSITION_1;
	
		HAL_UART_Transmit(&huart3, txBuffer, 16, HAL_MAX_DELAY);	
		return;
	
	case 2:
		floatUnion.f = Yaw;
		txBuffer[12] = floatUnion.bytes[0];
		txBuffer[13] = floatUnion.bytes[1];
		txBuffer[14] = floatUnion.bytes[2];
		txBuffer[15] = floatUnion.bytes[3];
		
		//CRC
		txBuffer[16] = 0;
		txBuffer[17] = 0;
		//包尾
		txBuffer[18] = FRAME_TAIL_POSITION_0;
		txBuffer[19] = FRAME_TAIL_POSITION_1;
	
		HAL_UART_Transmit(&huart3, txBuffer, 20, HAL_MAX_DELAY);	
		return;
	
	case 3:
		//CRC
		txBuffer[12] = 0;
		txBuffer[13] = 0;
		//包尾
		txBuffer[14] = FRAME_TAIL_POSITION_0;
		txBuffer[15] = FRAME_TAIL_POSITION_1;
	
		HAL_UART_Transmit(&huart3, txBuffer, 16, HAL_MAX_DELAY);	
		return;
	
	default:
		return;
	}
}





void POS_Change(float X, float Y)
{
	  //定义发送缓冲区
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






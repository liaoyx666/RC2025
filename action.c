#include "action.h"
#include <math.h>
ACTION_GL_POS ACTION_GL_POS_DATA = {0};
ROBOT_REAL_POS ROBOT_REAL_POS_DATA = {0};
/*
  @使用方法：
  使用串口3的232通道进行通讯，
  action数据存放在ACTION_GL_POS_DATA[]中，如果要调用请外部申明
  在.h文件中需要填入安装误差
  转换到车中心的坐标存放在ROBOT_REAL_POS_DATA[]中，如果要调用请外部申明
  更新XY坐标时需要调用Update_X，Update_Y。并且在两个函数中间要delay10ms
  
  此文件用于action而非position
*/



uint32_t Action_UART4_RxCallback(uint8_t *buf, uint16_t len)
{
	uint8_t count = 0;
	uint8_t i = 0;
	uint8_t id = 0;
	uint8_t length = 0;
	uint8_t CRC_HIGH = 0;
  uint8_t CRC_LOW = 0;
	union 
	{
		uint8_t data[12];
		float ActVal[3];
	} posture;
	
	uint8_t break_flag = 1;
	while(i < len && break_flag == 1)
	{
		switch (count)
		{
			//接收包头1
			case 0:
			{
				if (buf[i] == 0xfc)   
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
			//接收包头2
			case 1:
			{
				if (buf[i] == 0xfb) 
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
			//储存ID
			case 2:
			{
			    id = buf[i];
				  count++;
				  i++;
			}
			//储存数据长度
			case 3:
			{
			    length = buf[i];
				  i++;
			    count ++;
			}
			//开始接收数据，数据有X，Y，ANGEL_Z
			case 4:
			{
				uint8_t index;
				
				if (i > len - 26)
				{
					break_flag = 0;
				}
				//接收数据
				for(index = 0; index < 12; index++)
				{
					posture.data[index] = buf[i];
					i++;
				}
				count++;
				break;
			}
			
			case 5:
			{
				//接收CRC校验的高8位
			    CRC_HIGH = buf[i];
					i ++;
				  count ++;
			}
			case 6:
			{
				//接收CRC校验的低8位
			    CRC_LOW = buf[i];
				  i++;
				  count ++;
			}
			case 7:
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
			
			case 8:
			{
				if (buf[i] == 0xfe)  //接收包尾2
				{	
					//在接收包尾2后才开始处理数据
					Update_Action_gl_position(posture.ActVal);
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


void Update_Action_gl_position(float value[6])
{
	//赋值
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;

	//处理数据
	ACTION_GL_POS_DATA.ANGLE_Z = value[0];
	ACTION_GL_POS_DATA.POS_X = value[1]; 
	ACTION_GL_POS_DATA.POS_Y = value[2]; 

   //差分运算
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;

  //世界坐标中的
	ROBOT_REAL_POS_DATA.world_w = ACTION_GL_POS_DATA.ANGLE_Z;
  
	//加入安装误差
	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);
	
  //解算安装误差
	ROBOT_REAL_POS_DATA.world_x = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_X * sinf(ROBOT_REAL_POS_DATA.world_w * PI / 180.f);
	ROBOT_REAL_POS_DATA.world_y = ACTION_GL_POS_DATA.REAL_Y + INSTALL_ERROR_Y * cosf(ROBOT_REAL_POS_DATA.world_w * PI / 180.f);
}








void POS_Change(float X, float Y)
{
	  //定义接收缓冲区
    uint8_t txBuffer[10];
	//使用联合体以便将浮点型数据转换为字节并发送
    union {
        float f;
        uint8_t bytes[4];
    } floatUnion;
    // 起始标志
    txBuffer[0] = 0x02;  
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


   




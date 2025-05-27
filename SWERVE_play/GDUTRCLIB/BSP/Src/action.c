#include "action.h"
#include <math.h>

ACTION_GL_POS ACTION_GL_POS_DATA = {0};
ROBOT_REAL_POS ROBOT_REAL_POS_DATA = {0};
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

uint32_t Action_UART3_RxCallback(uint8_t *buf, uint16_t len)
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
			
			
			case 2:
				//接收帧ID和数据长度
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
				//开始接收数据
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



void Update_Action_gl_position(float value[5])
{
	//赋值
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;

	//处理数据

	ACTION_GL_POS_DATA.POS_X = value[0] / 1000; 
	ACTION_GL_POS_DATA.POS_Y = value[1] / 1000; 
	ACTION_GL_POS_DATA.ANGLE_Z = value[2];
	ACTION_GL_POS_DATA.Speed_X = value[3];
	ACTION_GL_POS_DATA.Speed_Y = value[4];

   //差分运算
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;

  //世界坐标
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







































//void Update_Action_gl_position(float value[6])
//{
//	
//	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
//	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;

//	
//	ACTION_GL_POS_DATA.ANGLE_Z = value[0];
//	ACTION_GL_POS_DATA.ANGLE_X = value[1];
//	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
//	ACTION_GL_POS_DATA.POS_X = value[3]; 
//	ACTION_GL_POS_DATA.POS_Y = value[4]; 
//	ACTION_GL_POS_DATA.W_Z = value[5];


//	
//	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
//	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;

//	

//   //差分运算
//	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
//	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;


//	ROBOT_REAL_POS_DATA.world_w = ACTION_GL_POS_DATA.ANGLE_Z;
//	//ROBOT_CHASSI.world_w = ROBOT_REAL_POS_DATA.world_w;


//	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
//	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);
//	

//	ROBOT_REAL_POS_DATA.world_x = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_X * sinf(ROBOT_REAL_POS_DATA.world_w * PI / 180.f);
//	//ROBOT_CHASSI.world_x = ROBOT_REAL_POS_DATA.world_x;
//	ROBOT_REAL_POS_DATA.world_y = ACTION_GL_POS_DATA.REAL_Y + INSTALL_ERROR_Y * cosf(ROBOT_REAL_POS_DATA.world_w * PI / 180.f);
//	//ROBOT_CHASSI.world_y = ROBOT_REAL_POS_DATA.world_y;
//}

////字符串拼接
//void Stract (char strDestination[],char strSource[] , int num)
//{
//    int i = 0 , j = 0;
//	  while(strDestination[i] != '\0') i++;
//	  
//	  for (j = 0; j < num ; j++)
//	        {
//			   strDestination[i++] = strSource[j];  
//			}
//}

////更新X坐标
//void Update_X(float New_X)
//{
//    char Update_x[8] = {'A', 'C', 'T', 'X', 0, 0, 0, 0};
//    
//    union
//    {
//        float X;
//        char data[4];
//    } New_set;

//    New_set.X = New_X;

//    Stract(&Update_x[4], New_set.data, 4);

//    HAL_UART_Transmit(&huart3, (uint8_t *)Update_x, 8, HAL_MAX_DELAY);
//}
////更新Y坐标
//void Update_Y(float New_Y)
//{
//    char Update_y[8] = {'A', 'C', 'T', 'Y', 0, 0, 0, 0};
//    
//    union
//    {
//        float Y;
//        char data[4];
//    } New_set;

//    New_set.Y = New_Y;

//    Stract(&Update_y[4], New_set.data, 4);

//    HAL_UART_Transmit(&huart3, (uint8_t *)Update_y, 8, HAL_MAX_DELAY);
//}




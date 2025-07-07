/**
 * @file		LaserPositioning_Task.c | LaserPositioning_Task.h
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-05-19 (��������)
 * @date        2025-05-30 (����޸�����)
 * @version     1.0.0
 * @note		�����ԣ����߲��Ƽ��ڵ�һ�����Ϲ��ض��������ģ��ʹ�ö����������Զ�����ģʽ���в�����ԭ��������
 *              1. ��ģʽ�£�������ģ����ĵ��β���ʱ���޷�ȷ����ֻ��ͨ��������ѯ�ķ�ʽ��ȡ���������������ʱ��Ŀ���
 *				2. ������ģ�����Ӧ��Ƶ�ʣ�ָģ���ڷ�����Ϣ���ÿ����ٴν���ָ���ʱ�䣩�����ƣ�����δ����
 * 				3. ��ʹ�ø�FreeRTOS����ϵͳʱ��δ֪ԭ����osDelay()��HAL_Delay()��������ʱ��׼ȷ�������޷������Ŀ��Ƶ�Ƭ����ģ�鷢��ָ���ʱ�������ᵼ��ģ���޷�������Ӧ��
 *				������δ���Թ���һ�����Ϲ��ض��������ģ��������е����Զ�����ģʽ�Ĳ���
 *				����ǽ���ʹ�öമ�ڷֱ���ص���������ģ��ķ�ʽ���в���
 *				������ʵҲ����������̫�ˣ����Ե��������ϲ�������Ĳ���
 * @warning		�����������޸�LaserModuleGroup_Init()�����������Ǻ��εĳ�����ʵ�ϣ�����Ҳ��֪��Ϊʲô��������ͨ
 * @license     WTFPL License
 *
 * @par �汾�޶���ʷ
 * @{
 *  @li �汾��: 1.0.0
 *		- �޶�����: 2025-05-30
 *		- ��Ҫ���:
 *			- �������������ģ�������������
 *		- ����֮��:
 *			- ģ���״̬�����ò����ƣ���״̬���Ĵ���Ҳ��������
 *			- �����ڳ�ʼ�������еĽ�׳�Բ���
 * 			- �����Yaw������������������������
 *		- ����:
 *			- ����������������д
 *		- ����: ZhangJiaJia
 * 
 *  @li �汾��: 0.3.1
 *      - �޶�����: 2025-05-27
 *      - ��Ҫ���:
 *			- �޸���CubeMX�����е�һЩ�޺�bug
 *      - ����: ZhangJiaJia
 *
 *	@li �汾��: 0.3.0
 *		- �޶�����: 2025-05-24
 *		- ��Ҫ���:
 *			- ��2.0�汾δ��ɵ���������ģ������˲��Բ�ͨ��
 *			- ��д����˼򵥵Ķ�λ�㷨����
 *			- �޸���0.2.1�汾δ��ȫ�޸����޺�bug
 *		- ����֮��:
 *			- ������ʵ������
 *		- ����: ZhangJiaJia
 *
 *  @li �汾��: 0.2.1
 *		- �޶�����: 2025-05-23
 *		- ��Ҫ���:
 *			- ʹ��vTaskDelayUntil()������ osDelay() �����������Ż�
 *			- �޸���FreeRTOS�����ļ���������������޺�bug
 *		- ����: ZhangJiaJia
 *
 *	@li �汾��: 0.2.0
 *		- �޶�����: 2025-05-23
 *      - ��Ҫ���:
 *			- ʵ���˼�����ģ����Ķ����������Զ���������ȡ����״̬����ȡ��������ĺ���
 *			- ����˼��׵�ģ��״̬��
 *		- ����֮��:
 *			- Ŀǰֻ�Ե�������ģ������˲��ԣ�δͬʱ����������ģ����в���
 *			- ��ʱ���������û�������Ҫ��һ��ʹ��vTaskDelayUntil()���������Ż�
 *			- ģ���״̬�����ò����ƣ���״̬���Ĵ���Ҳ��������
 *      - ����: ZhangJiaJia
 *
 *	@li �汾��: 0.1.0
 *      - �޶�����: 2025-05-21
 *      - ��Ҫ���:
 *			- �½� LaserPositioning_Task ���������uart4��DMA�����жϽ��ճ��򣬲������յ����ݴ���FreeRTOS�Ķ�����
 *      - ����: ZhangJiaJia
 * @}
 */


// ��������ϵ���壺
// �������������������ص����ϽǶ���Ϊ����ԭ�㣬����ΪX�ᣬ����ΪY�ᣬ
// ��������ϵ��X�᷽��Ϊ0����ʱ��Ϊ������Ĭ�ϵ�λ���ȣ���Χ��-PI��PI֮��

// ״̬����0���������������쳣

// ������ģ��1������UART3
// ������ģ��2������UART4


#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"		// main.h ͷ�ļ�������ʵ�Ѿ����������ͷ�ļ�
#include "cmsis_os.h"
#include "data_pool.h"
#include "FreeRTOS.h"
#include "main.h"
#include "LaserPositioning.h"


#define LaserPositionin_UART_SIZE 15


#define PI							3.14159265358979323846f			// ����Բ���ʳ���PI


#define LaserModule_1_UartHandle &huart4		// ������ģ��1���ھ��
#define LaserModule_2_UartHandle &huart5		// ������ģ��2���ھ��
#define LaserModule_3_UartHandle &huart6		// ������ģ��2���ھ��

#define LaserModule1Address				0x00							// ������ģ��1��ַ
#define LaserModule1ReadAddress			(LaserModule1Address | 0x80)	// ������ģ��1����ַ
#define LaserModule1WriteAddress		LaserModule1Address				// ������ģ��1д��ַ

#define LaserModule2Address				0x00							// ������ģ��2��ַ
#define LaserModule2ReadAddress			(LaserModule2Address | 0x80)	// ������ģ��2����ַ
#define LaserModule2WriteAddress		LaserModule2Address 			// ������ģ��2д��ַ

#define LaserModule3Address				0x00							// ������ģ��3��ַ
#define LaserModule3ReadAddress			(LaserModule2Address | 0x80)	// ������ģ��3����ַ
#define LaserModule3WriteAddress		LaserModule2Address 			// ������ģ��3д��ַ

int16_t FrontLaserDistanceOffset	= 0;			// ǰ���ⰲװ����ƫ��������λ��mm
int16_t RightLaserDistanceOffset	= 0;			// �Ҽ��ⰲװ����ƫ��������λ��mm
float YawOffset					= 0.0f;			// ƫ����ƫ��������λ����
//uint16_t FrontLaserAngleOffset_ActualDistance		= 0;		// ǰ���ⰲװ�Ƕ�ƫ����_ʵ�ʾ��룬��λ��mm
int16_t FrontLaserAngleOffset_OffsetDistance		= 0;		// ǰ���ⰲװ�Ƕ�ƫ����_ƫ�ƾ��룬��λ��mm
//uint16_t RightLaserAngleOffset_ActualDistance		= 0;		// �Ҽ��ⰲװ�Ƕ�ƫ����_ʵ�ʾ��룬��λ��mm
int16_t RightLaserAngleOffset_OffsetDistance		= 0;		// �Ҽ��ⰲװ�Ƕ�ƫ����_ƫ�ƾ��룬��λ��mm


static uint8_t LaserPositionin_Rx_Buff[LaserPositionin_UART_SIZE];
LaserModuleDataGroupTypedef LaserModuleDataGroup;		// ������ģ�����������


static uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModule_StateContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData);
static uint8_t LaserModuleGroup_AnalysisModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
static uint8_t LaserModule_AnalysisModulesMeasurementResults(LaserModuleDataTypedef* LaserModuleData);
static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw, uint32_t FrontLaser, uint32_t RightLaser);
static void LaserPositioning_GetYaw(float* Yaw);
static void LaserPositioning_SendXYWorldCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates);
static uint8_t MyUART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size);


void LaserPositioning(float Yaw, WorldXYCoordinatesTypedef* WorldXYCoordinatesPointer)
{
	uint8_t LaserModuleGroupState = 0;	// ������ģ��״̬����


	LaserModuleGroupState = 0;	// ������ģ��״̬����
	LaserModuleDataGroup.LaserModule1.MeasurementData.State = 0;	// ������ģ��1״̬����
	LaserModuleDataGroup.LaserModule2.MeasurementData.State = 0;	// ������ģ��2״̬����
	LaserModuleDataGroup.LaserModule3.MeasurementData.State = 0;	// ������ģ��3״̬����

	LaserModuleGroupState |= LaserModuleGroup_AnalysisModulesMeasurementResults(&LaserModuleDataGroup);			// ������ģ�����ȡ�������

	//LaserPositioning_GetYaw(&Yaw);		// ��ȡƫ���ǣ���λ����
	
	//LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesPointer, Yaw, LaserModuleDataGroup.LaserModule1.MeasurementData.Distance, LaserModuleDataGroup.LaserModule2.MeasurementData.Distance);

	//LaserPositioning_SendXYWorldCoordinates(WorldXYCoordinatesPointer);	// ������������ϵXY��������
}

void LaserModuleGroup_Init(void)
{
	uint8_t LaserModuleGroupState = 0;		// ������ģ��״̬����

	// ������ģ��1��ʼ��
	LaserModuleDataGroup.LaserModule1.ConfigurationData.UartHandle = LaserModule_1_UartHandle;		// ���ü�����ģ��1�Ĵ��ھ��
	LaserModuleDataGroup.LaserModule1.ConfigurationData.ReceiveQueue = Receive_LaserModuleData_1_Port;	// ���ü�����ģ��1�Ĵ���DMA���ն���
	LaserModuleDataGroup.LaserModule1.ConfigurationData.Address = LaserModule1Address;
	LaserModuleDataGroup.LaserModule1.ConfigurationData.ReadAddress = LaserModule1ReadAddress;
	LaserModuleDataGroup.LaserModule1.ConfigurationData.WriteAddress = LaserModule1WriteAddress;
	LaserModuleDataGroup.LaserModule1.MeasurementData.Distance = 0;	// ������ģ��1�������ݳ�ʼ��
	LaserModuleDataGroup.LaserModule1.MeasurementData.SignalQuality = 0;	// ������ģ��1�ź��������ݳ�ʼ��
	LaserModuleDataGroup.LaserModule1.MeasurementData.State = 0;	// ������ģ��1״̬���ݳ�ʼ��

	// ������ģ��2��ʼ��
	LaserModuleDataGroup.LaserModule2.ConfigurationData.UartHandle = LaserModule_2_UartHandle;		// ���ü�����ģ��2�Ĵ��ھ��
	LaserModuleDataGroup.LaserModule2.ConfigurationData.ReceiveQueue = Receive_LaserModuleData_2_Port;	// ���ü�����ģ��2�Ĵ���DMA���ն���
	LaserModuleDataGroup.LaserModule2.ConfigurationData.Address = LaserModule2Address;
	LaserModuleDataGroup.LaserModule2.ConfigurationData.ReadAddress = LaserModule2ReadAddress;
	LaserModuleDataGroup.LaserModule2.ConfigurationData.WriteAddress = LaserModule2WriteAddress;
	LaserModuleDataGroup.LaserModule2.MeasurementData.Distance = 0;	// ������ģ��2�������ݳ�ʼ��
	LaserModuleDataGroup.LaserModule2.MeasurementData.SignalQuality = 0;	// ������ģ��2�ź��������ݳ�ʼ��
	LaserModuleDataGroup.LaserModule2.MeasurementData.State = 0;	// ������ģ��2״̬���ݳ�ʼ��
	
	// ������ģ��3��ʼ��
	LaserModuleDataGroup.LaserModule3.ConfigurationData.UartHandle = LaserModule_3_UartHandle;		// ���ü�����ģ��3�Ĵ��ھ��
	LaserModuleDataGroup.LaserModule3.ConfigurationData.ReceiveQueue = Receive_LaserModuleData_3_Port;	// ���ü�����ģ��3�Ĵ���DMA���ն���
	LaserModuleDataGroup.LaserModule3.ConfigurationData.Address = LaserModule3Address;
	LaserModuleDataGroup.LaserModule3.ConfigurationData.ReadAddress = LaserModule3ReadAddress;
	LaserModuleDataGroup.LaserModule3.ConfigurationData.WriteAddress = LaserModule3WriteAddress;
	LaserModuleDataGroup.LaserModule3.MeasurementData.Distance = 0;	// ������ģ��3�������ݳ�ʼ��
	LaserModuleDataGroup.LaserModule3.MeasurementData.SignalQuality = 0;	// ������ģ��3�ź��������ݳ�ʼ��
	LaserModuleDataGroup.LaserModule3.MeasurementData.State = 0;	// ������ģ��3״̬���ݳ�ʼ��

	//LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup.LaserModule1);	// �򿪼�����ģ��1�ļ�����
	//LaserModuleGroupState |= LaserModule_TurnOnTheLaserPointer(&LaserModuleDataGroup.LaserModule2);	// �򿪼�����ģ��2�ļ�����

	//taskENTER_CRITICAL();

	HAL_Delay(3000);
	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(&LaserModuleDataGroup.LaserModule3);	// ������ģ��3�����Զ�����״̬����
	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(&LaserModuleDataGroup.LaserModule2);	// ������ģ��2�����Զ�����״̬����
	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(&LaserModuleDataGroup.LaserModule1);	// ������ģ��1�����Զ�����״̬����

	//taskEXIT_CRITICAL();

}

static uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleGroupState = 0;

	// ���ô򿪼�����������
	uint8_t CMD[9] = { 0xAA, LaserModuleData->ConfigurationData.WriteAddress, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x01, 0x00 };
	uint8_t CheckValueCalculation = CMD[1] + CMD[2] + CMD[3] + CMD[4] + CMD[5] + CMD[6] + CMD[7];
	CMD[8] = CheckValueCalculation;

	LaserModuleGroupState |= MyUART_Transmit_DMA(LaserModuleData->ConfigurationData.UartHandle, CMD, sizeof(CMD));		// ���ʹ򿪼�����������

	if (xQueueReceive(LaserModuleData->ConfigurationData.ReceiveQueue, LaserPositionin_Rx_Buff, pdMS_TO_TICKS(50)) == pdPASS)	// �ȴ����ռ�����ģ���Ӧ������
	{
		if (strcmp(LaserPositionin_Rx_Buff, CMD) == 0)		// �ԱȽ��յ����ݺͷ��͵�����
		{
			return 0;	// ������ģ��״̬����
		}
		else
		{
			LaserModuleData->MeasurementData.State |= 0x02;		// ������ģ���ʼ�����󣬴���ԭ�򣬽������ݰ��ȶ�У�鲻ͨ��
			return 1;	// ������ģ��״̬�쳣
		}
	}
	else
	{
		LaserModuleData->MeasurementData.State |= 0x01;	// ������ģ���ʼ�����󣬴���ԭ�򣬽������ݰ��ȴ���ʱ
		return 1;	// ������ģ��״̬�쳣
	}

	return LaserModuleGroupState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserModule_StateContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;	// ������ģ��״̬����

	// ���������Զ�����������
	uint8_t CMD[9] = { 0xAA, LaserModuleData->ConfigurationData.WriteAddress, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x00 };
	uint8_t CheckValueCalculation = CMD[1] + CMD[2] + CMD[3] + CMD[4] + CMD[5] + CMD[6] + CMD[7];
	CMD[8] = CheckValueCalculation;

	LaserModuleState |= MyUART_Transmit_DMA(LaserModuleData->ConfigurationData.UartHandle, CMD, sizeof(CMD));		// �������������Զ�����ģ�������

	return LaserModuleState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserModuleGroup_AnalysisModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// ������ģ��״̬����

	LaserModuleGroupState |= LaserModule_AnalysisModulesMeasurementResults(&LaserModuleDataGroup->LaserModule1);	// ������ģ��1��ȡ�������
	LaserModuleGroupState |= LaserModule_AnalysisModulesMeasurementResults(&LaserModuleDataGroup->LaserModule2);	// ������ģ��2��ȡ�������
	LaserModuleGroupState |= LaserModule_AnalysisModulesMeasurementResults(&LaserModuleDataGroup->LaserModule3);	// ������ģ��3��ȡ�������

	return LaserModuleGroupState;			// ���ؼ�����ģ��״̬
}

static uint8_t LaserModule_AnalysisModulesMeasurementResults(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;		// ������ģ��״̬����

	if (xQueueReceive(LaserModuleData->ConfigurationData.ReceiveQueue, LaserPositionin_Rx_Buff, pdFALSE) == pdPASS)
	{
		uint32_t Distance =
			(LaserPositionin_Rx_Buff[6] << 24) |
			(LaserPositionin_Rx_Buff[7] << 16) |
			(LaserPositionin_Rx_Buff[8] << 8) |
			(LaserPositionin_Rx_Buff[9] << 0);		// ���ղ��������
	
		uint16_t SignalQuality =
			(LaserPositionin_Rx_Buff[10] << 8) |
			(LaserPositionin_Rx_Buff[11] << 0);		// ���ղ������ź�����
	
		uint8_t CheckValueReceive = LaserPositionin_Rx_Buff[12];	// ����У��ֵ
	
		uint8_t CheckValueCalculation = 0;
		for (uint8_t i = 1; i < 12; i++)
		{
			CheckValueCalculation += LaserPositionin_Rx_Buff[i];		// ����У��ֵ
		}
	
		if (CheckValueReceive == CheckValueCalculation)
		{
			LaserModuleData->MeasurementData.Distance = Distance;				// ���¼�����ģ��1�ľ�������
			LaserModuleData->MeasurementData.SignalQuality = SignalQuality;
		}
		else
		{
			LaserModuleData->MeasurementData.State |= 0x04;		// ������ģ��������󣬴���ԭ�򣬽������ݰ�У��λ��ͨ��
			LaserModuleState |= 0x01;							// ������ģ��״̬�쳣
		}
	}
	else
	{
		//LaserModuleData->MeasurementData.State |= 0x08;		// ������ģ��������󣬴���ԭ��δ���յ����ݰ�
		//LaserModuleState |= 0x01;							// ������ģ��״̬�쳣
	}

	return LaserModuleState;			// ���ؼ�����ģ��״̬
}

static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw, uint32_t FrontLaser, uint32_t RightLaser)
{
	FrontLaser += FrontLaserDistanceOffset;		// ǰ���ⰲװ����ƫ����У������λ��mm
	RightLaser += RightLaserDistanceOffset;		// �Ҽ��ⰲװ����ƫ����У������λ��mm
	Yaw += ((float)YawOffset * PI / 180.0f);	// ƫ����ƫ����У��
	
	//float FrontLaserAngleOffset = atan((float)FrontLaserAngleOffset_OffsetDistance / (float)FrontLaserAngleOffset_ActualDistance);
	//float RightLaserAngleOffset = atan(((float)(-RightLaserAngleOffset_OffsetDistance)) / (float)RightLaserAngleOffset_ActualDistance);

	float FrontLaserAngleOffset = asinf((float)FrontLaserAngleOffset_OffsetDistance / (float)FrontLaser);
	float RightLaserAngleOffset = asinf(((float)(-RightLaserAngleOffset_OffsetDistance)) / (float)RightLaser);

	WorldXYCoordinates->Y = -(((float)FrontLaser * sinf(Yaw - FrontLaserAngleOffset)) / 1000.0);
	WorldXYCoordinates->X = -(((float)RightLaser * sinf(Yaw - RightLaserAngleOffset)) / 1000.0);
}

static void LaserPositioning_GetYaw(float* Yaw)
{
	//if (xQueueReceive(Receive_LaserPositioning_Yaw_Port, Yaw, pdFALSE) == pdPASS)
	//{
	//	// ����ƫ���ǳɹ�
	//	// Yaw ��ֵ�Ѿ��ڽ���ʱ������
	//}
	//else
	//{
	//	// ����ƫ����ʧ��
	//	// ���־ɵ�ƫ����ֵ����
	//}

	//*Yaw = g_LaserPositioning_Yaw;		// ��ȡƫ���ǣ���λ����
}

static void LaserPositioning_SendXYWorldCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates)
{
	//if (xQueueOverwrite(Send_LaserPositioning_XYWorldCoordinates_Port, WorldXYCoordinates, &xHigherPriorityTaskWoken) == pdPASS)
	//{
	//	// �����������л�������Ҫ��
	//	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	//	return 1;   // ���ͳɹ�
	//}
	//else
	//{
	//	return 0;   // ���з���ʧ��
	//}
}

static uint8_t MyUART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size)
{
	HAL_StatusTypeDef UART_Status = HAL_OK;

	UART_Status = HAL_UART_Transmit(huart, pData, Size,100);

	if (UART_Status == HAL_OK)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief	    ������ģ��6����DMA������ϻص�����
 * @param[in]	Receive_data ����DMA���ջ��������ַ
 * @param[in]   data_len ����DMA���ջ���������󳤶�
 * @return		uint32_t �ص�������������������״̬��1 Ϊ���ͳɹ���0 Ϊ����ʧ��
 * @note		�����ʦ�ִ��Ĵ��룬�Ҳ����returnֵΪʲô��uint32_t����
 */
uint32_t LaserPositionin_UART6_RxCallback(uint8_t* buff, uint16_t len)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (xQueueOverwriteFromISR(Receive_LaserModuleData_3_Port, buff, &xHigherPriorityTaskWoken) == pdPASS)
	{
		// �����������л�������Ҫ��
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		return 1;   // ���ͳɹ�
	}
	else
	{
		return 0;   // ���з���ʧ��
	}
	return 0;
}

/**
 * @brief	    ������ģ��2����DMA������ϻص�����
 * @param[in]	Receive_data ����DMA���ջ��������ַ
 * @param[in]   data_len ����DMA���ջ���������󳤶�
 * @return		uint32_t �ص�������������������״̬��1 Ϊ���ͳɹ���0 Ϊ����ʧ��
 * @note		�����ʦ�ִ��Ĵ��룬�Ҳ����returnֵΪʲô��uint32_t����
 */
uint32_t LaserPositionin_UART5_RxCallback(uint8_t* buff, uint16_t len)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (xQueueOverwriteFromISR(Receive_LaserModuleData_2_Port, buff, &xHigherPriorityTaskWoken) == pdPASS)
	{
		// �����������л�������Ҫ��
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		return 1;   // ���ͳɹ�
	}
	else
	{
		return 0;   // ���з���ʧ��
	}
	return 0;
}

/**
 * @brief	    ������ģ��1����DMA������ϻص�����
 * @param[in]	Receive_data ����DMA���ջ��������ַ
 * @param[in]   data_len ����DMA���ջ���������󳤶�
 * @return		uint32_t �ص�������������������״̬��1 Ϊ���ͳɹ���0 Ϊ����ʧ��
 * @note		�����ʦ�ִ��Ĵ��룬�Ҳ����returnֵΪʲô��uint32_t����
 */
uint32_t LaserPositionin_UART4_RxCallback(uint8_t* buff, uint16_t len)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (xQueueOverwriteFromISR(Receive_LaserModuleData_1_Port, buff, &xHigherPriorityTaskWoken) == pdPASS)
	{
		// �����������л�������Ҫ��
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		return 1;   // ���ͳɹ�
	}
	else
	{
		return 0;   // ���з���ʧ��
	}
	return 0;
}
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


#ifndef __LaserPositioning_Task_H
#define __LaserPositioning_Task_H


#include <stdint.h>
#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "data_pool.h"

// C���Բ���
#ifdef __cplusplus
extern "C" {
#endif


// �Ժ�������ֵ LaserModuleGroupState ��˵����
// 0x00������ģ���鴦������״̬
// 0x01������ģ���鴦���쳣״̬

// �� LaserModuleMeasurementDataTypedef �е� State ��˵����
// 0x00������ģ�鴦������״̬
// 0x01��������ģ���ʼ�����󣬴���ԭ�򣬽������ݰ��ȴ���ʱ
// 0x02��������ģ���ʼ�����󣬴���ԭ�򣬽������ݰ��ȶ�У�鲻ͨ��
// 0x04��������ģ��������󣬴���ԭ�򣬽������ݰ�У��λ��ͨ��
// 0x08����


typedef struct LaserModuleConfigurationData
{
	UART_HandleTypeDef* UartHandle;			// ���ھ��
	QueueHandle_t ReceiveQueue;		// ����DMA���ն��о��
	uint8_t Address;			// ����ģ��ԭʼ��ַ
	uint8_t ReadAddress;
	uint8_t WriteAddress;
}LaserModuleConfigurationDataTypedef;

typedef struct LaserModuleMeasurementData
{
	uint32_t Distance;
	uint16_t SignalQuality;
	uint16_t State;
}LaserModuleMeasurementDataTypedef;

typedef struct LaserModuleData
{
	LaserModuleConfigurationDataTypedef ConfigurationData;
	LaserModuleMeasurementDataTypedef MeasurementData;
}LaserModuleDataTypedef;

typedef struct LaserModuleDataGroup
{
	LaserModuleDataTypedef LaserModule1;
	LaserModuleDataTypedef LaserModule2;
}LaserModuleDataGroupTypedef;


typedef struct WorldXYCoordinates
{
	float X;		// ��λ��m
	float Y;		// ��λ��m
}WorldXYCoordinatesTypedef;


void LaserModuleGroup_Init(void);
void LaserPositioning_Task(float* YawPointer, WorldXYCoordinatesTypedef* WorldXYCoordinatesPointer);

uint32_t LaserPositionin_UART2_RxCallback(uint8_t* Receive_data, uint16_t data_len);    // ������ģ��1���ڽ��ջص�����
uint32_t LaserPositionin_UART6_RxCallback(uint8_t* Receive_data, uint16_t data_len);    // ������ģ��2���ڽ��ջص�����


#ifdef __cplusplus
}
#endif


#endif

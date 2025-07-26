/**
 * @file    drive_atk_mw1278d_uart.h
 * @brief   ATK-MW1278D ģ�� UART ����ͷ�ļ�
 * @author  �����������
 * @date    2025-07-11
 */

#ifndef __DRIVE_ATK_MW1278D_UART_H
#define __DRIVE_ATK_MW1278D_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* ������ͷ�ļ� ----------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* �궨�� ----------------------------------------------------------------*/
#define ATK_MW1278D_UART_RX_BUF_SIZE    128    /* UART���ջ�������С */

/* �������Ͷ��� ----------------------------------------------------------*/
/**
 * @brief ���ݻ������ṹ��
 */
typedef struct {
    float num1[3];      /* ��һ�����ݻ��������洢float�������� */
    float num2[3];      /* �ڶ������ݻ��������洢float�������� */
    int   num3[3];      /* ���������ݻ��������洢int�������� */
} ATK_MW1278D_DataBufferTypeDef;

/* �������� --------------------------------------------------------------*/
/**
 * @brief  ��ATK-MW1278Dģ�鷢�͸�ʽ������
 * @param  fmt: ��ʽ���ַ���
 * @param  ...: �ɱ�����б�
 * @retval ��
 */
void atk_mw1278d_uart_printf(char *fmt, ...);

/**
 * @brief  ��ȡATK-MW1278Dģ������ݻ�����
 * @retval ���ݻ������ṹ��ָ��
 */
ATK_MW1278D_DataBufferTypeDef* atk_mw1278d_get_data_buffer(void);

/**
 * @brief  ������½��յ��������Ƿ���Ч
 * @retval ������Ч��״̬��1-��Ч��0-��Ч
 */
int atk_mw1278d_is_data_valid(void);

/**
 * @brief  UART���ջص����������ڴ�����յ�������
 * @param  buf: �������ݻ�����
 * @param  len: �������ݳ���
 * @retval ������ֽ����������
 */
uint32_t Lora_UART2_RxCallback(uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVE_ATK_MW1278D_UART_H */
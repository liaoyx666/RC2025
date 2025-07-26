#include "drive_atk_mw1278d_uart.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief       ATK_MW1278D UART printf
 * @param       fmt: ����ӡ������
 * @retval      ��
 */
 
float valid_num1;
float valid_num2;
float valid_num3;

#define SEND_BUF_SIZE1 100
uint8_t Sendbuf1[SEND_BUF_SIZE1];

/**
 * @brief ����
 */
void atk_mw1278d_uart_printf(char *fmt, ...)
{
    memset(Sendbuf1, 0, SEND_BUF_SIZE1);  // ��շ��ͻ�����
    
    va_list arg;
    va_start(arg, fmt);
    vsnprintf((char*)Sendbuf1 + 1, SEND_BUF_SIZE1 - 2, fmt, arg);  // �����ռ����ͷ�Ͱ�β
    va_end(arg);
    
    // ��Ӱ�ͷ�Ͱ�β
    Sendbuf1[0] = '@';  // ��ͷ
    
    uint8_t len = strlen((char*)Sendbuf1);  // ����ʵ���ַ�������
    Sendbuf1[len] = '?';  // ��β
    len++;  // ���³����԰�����β
    
    if(len > 2)  // ȷ�������а�ͷ�Ͱ�β
    {
        HAL_UART_Transmit_DMA(&huart2, Sendbuf1, len);  // ͨ��DMA�����ַ���
    }
	
}

// ȫ�����ݻ�����(����ջռ��)
static struct {
    float num1[3];
    float num2[3];
    int   num3[3];
} g_data_buf = {0};



// ȫ��״̬����
static uint8_t g_idx = 0;            // ���λ���������(1�ֽ�)
static uint8_t g_count = 0;          // ��Ч���ݼ���(1�ֽ�)
static float   g_num1, g_num2;       // ��ʱ�������
static int     g_num3;
static char    g_parse_buf[32];      // �����û�����
static uint8_t *g_buf;
static uint16_t g_len;
static int      g_valid;

// ʾ�����ݺͷ��ͻ�����(�ϲ���������ڴ���Ƭ)
    int temp=25545;
    float flt1=581.575747f;
    float flt2=3.1415926f;

// �Ż��ĸ�����ת�ַ�������(����ջʹ��)
static uint8_t float_to_str(char *buf, float value, uint8_t decimals) {
    int32_t integer_part = (int32_t)value;
    int32_t fractional_part = (int32_t)((value - integer_part) * 
                            (decimals == 1 ? 10 : decimals == 2 ? 100 : 1000));
                            
    uint8_t len = 0;
    char temp[16];
    int32_t num = integer_part;
    int8_t i = 0;
    
    // ������
    if (num < 0) {
        *buf++ = '-';
        len++;
        num = -num;
    }
    
    // ������������
    if (num == 0) {
        temp[i++] = '0';
    } else {
        while (num > 0) 
        {
            temp[i++] = (num % 10) + '0';
            num /= 10;
        }
    }
    
    // ��ת��������
    while (i > 0) {
        *buf++ = temp[--i];
        len++;
    }
    
    // ����С������
    if (decimals > 0) {
        *buf++ = '.';
        len++;
        
        // ȷ��С���������㹻��λ��
        if (decimals == 2) {
            *buf++ = (fractional_part / 10) % 10 + '0';
            *buf++ = fractional_part % 10 + '0';
            len += 2;
        } else if (decimals == 1) {
            *buf++ = fractional_part % 10 + '0';
            len++;
        }
    }
    
    return len;
}


// ���Ŷ��ж�(�������ٵ��ÿ���)
static inline int is_valid_data(void) {
    // ֻ������µ��������ݵ㣬���ټ�����
    uint8_t prev_idx = (g_idx + 2) % 3;  // ǰһ������
    
    // ���num1
    float diff_f = (g_data_buf.num1[g_idx] > g_data_buf.num1[prev_idx]) ?
                  (g_data_buf.num1[g_idx] - g_data_buf.num1[prev_idx]) :
                  (g_data_buf.num1[prev_idx] - g_data_buf.num1[g_idx]);
    if(diff_f >= 100) return 0;
    
    // ���num2
    diff_f = (g_data_buf.num2[g_idx] > g_data_buf.num2[prev_idx]) ?
            (g_data_buf.num2[g_idx] - g_data_buf.num2[prev_idx]) :
            (g_data_buf.num2[prev_idx] - g_data_buf.num2[g_idx]);
    if(diff_f >= 100) return 0;
    
    // ���num3
    int diff_i = (g_data_buf.num3[g_idx] > g_data_buf.num3[prev_idx]) ?
                (g_data_buf.num3[g_idx] - g_data_buf.num3[prev_idx]) :
                (g_data_buf.num3[prev_idx] - g_data_buf.num3[g_idx]);
    if(diff_i >= 100) return 0;
    
    return 1;
}


// �Ż��Ľ�������(�������ٵ��ÿ���)
// �����Ľ�������
static inline void parse_data(uint8_t *buf, uint16_t len) {
    char *p = (char*)buf;
    char *start = p;
    
    // ���Ƹ��Ƴ��ȣ��������
    len = (len > 31) ? 31 : len;
    memcpy(g_parse_buf, p, len);
    g_parse_buf[len] = '\0';
    
    // ����num1
    g_num1 = atoi(start);
    
    // ���ҵ�һ������
    p = strchr(start, ',');
    if (!p) {
        // û���ҵ����ţ�����ʧ��
        g_num2 = 0;
        g_num3 = 0;
        return;
    }
    
    // ����num2
    p++; // ��������
    g_num2 = atoi(p);
    
    // ���ҵڶ�������
    p = strchr(p, ',');
    if (!p) {
        // û���ҵ��ڶ������ţ�����ʧ��
        g_num3 = 0;
        return;
    }
    
    // ����num3
    p++; // ��������
    g_num3 = atoi(p);
}


// ����״̬��ö��
typedef enum {
    RX_STATE_IDLE,      // ����״̬
    RX_STATE_HEADER,    // �ѽ��յ���ͷ
    RX_STATE_DATA,      // ��������
    RX_STATE_TAIL       // �ȴ���β
} RX_STATE;

static RX_STATE g_rx_state = RX_STATE_IDLE;  // ����״̬��
static uint8_t g_rx_buf[128];                // ���ջ�����
static uint8_t g_rx_len = 0;                 // ��ǰ���ճ���

// ���ջص����������������ݲ�����
uint32_t Lora_UART2_RxCallback(uint8_t *buf, uint16_t len) 
{
    uint8_t i = 0;
    uint8_t frame_start = 0;
    uint8_t frame_found = 0;
    
    // �����������������ݰ�
    while (i < len) 
    {
        // ����֡ͷ '@'
        if (buf[i] == '@') 
        {
            frame_start = i;
            frame_found = 0;
            
            // ���Ҷ�Ӧ��֡β '?'
            for (uint8_t j = i + 1; j < len; j++) 
            {
                if (buf[j] == '?') {
                    // �ҵ�������֡: @...?
                    uint16_t frame_len = j - frame_start - 1;
                    
                    // ����֡���ݣ�����֡ͷ��֡β��
                    if (frame_len > 0) {
                        g_buf = &buf[frame_start + 1];
                        g_len = frame_len;
                        
                        // ��������
                        parse_data(g_buf, g_len);
                        
                        // ����������
                        g_data_buf.num1[g_idx] = g_num1;
                        g_data_buf.num2[g_idx] = g_num2;
                        g_data_buf.num3[g_idx] = g_num3;
                        
                        // ��������
                        g_idx = (g_idx + 1) % 3;
                        if(g_count < 3) g_count++;
                        
                        // ���������Ч��
                        g_valid = is_valid_data();
                        
  
						valid_num1=g_num1;
						valid_num2=g_num2;
						valid_num3=g_num3;
                    }
                    
                    i = j + 1;  // ����������һ֡
                    frame_found = 1;
                    break;
                }
            }
            
            if (!frame_found) {
                // û���ҵ�ƥ���֡β���˳�ѭ��
                break;
            }
        } else {
            i++;  // ��������֡ͷ
        }
    }

    return 0;  // ���ش�����ֽ����������
}
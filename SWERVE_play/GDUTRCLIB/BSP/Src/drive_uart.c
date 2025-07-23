/**
 * @file drive_uart.c
 * @author YangJianyi
 * @brief 1)串口底层驱动文件，使用该文件，需要在cubeMX中配置好串口硬件(参考大疆电机所需的串口的配置)，并在main.c中调用Uart_Init函数进行初始化
 *        2)默认使用串口DMA接收，当接收到数据时，会调用Uart_Rx_Idle_Callback函数。
 *        
 *        3)学院的串口协议默认使用以下格式（示例可查阅ROS.cpp文件）：
 *          前两位:包头: 0x55 0xAA     
 *          第三位:数据长度: 1字节(总的buf长度-6)
 *          倒数第四位:crc8校验位: 1字节
 *          末尾两位:包尾: 0x0D 0x0A
 * 
 *************************************************************************************************************************
 * 注意!!!!!!!!!!!
 *          使用该文件需要在stm32f4xx_it.c中的串口中断服务函数中添加中断接收函数，例如:Uart_Receive_Handler(&usart1_manager);
 *          该文件中包含了串口1、串口2、串口3、串口6的回调函数，如有需要，可自行添加其他串口
 * *************************************************************************************************************************
 * @version 0.1
 * @date 2024-04-03
 * 
 */

#include "drive_uart.h"

usart_manager_t usart1_manager = {.call_back_fun = NULL};
usart_manager_t usart2_manager = {.call_back_fun = NULL};
usart_manager_t usart3_manager = {.call_back_fun = NULL};
usart_manager_t usart6_manager = {.call_back_fun = NULL}; 
usart_manager_t usart5_manager = {.call_back_fun = NULL}; 
usart_manager_t usart4_manager = {.call_back_fun = NULL}; 

static void Uart_Rx_Idle_Callback(usart_manager_t *manager);

void Uart_Init(UART_HandleTypeDef *huart, uint8_t *Rxbuffer, uint16_t len, usart_call_back call_back_fun)
{
    if(huart == NULL)
        Error_Handler();
    else{}

    if(huart->Instance == USART1)
    {
        usart1_manager.uart_handle = huart;
        usart1_manager.rx_buffer = Rxbuffer;
        usart1_manager.rx_buffer_size = len;
        Usart_Rx_Callback_Register(&usart1_manager, call_back_fun);
        __HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		HAL_UART_Receive_DMA(huart, Rxbuffer, len);
    }
//    else if(huart->Instance == USART2)
//    {
//        usart2_manager.uart_handle = huart;
//        usart2_manager.rx_buffer = Rxbuffer;
//        usart2_manager.rx_buffer_size = len;
//        Usart_Rx_Callback_Register(&usart2_manager, call_back_fun);
//        __HAL_UART_CLEAR_IDLEFLAG(huart);
//		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
//		HAL_UART_Receive_DMA(huart, Rxbuffer, len);
//    }
    else if(huart->Instance == USART3)
    {
        usart3_manager.uart_handle = huart;
        usart3_manager.rx_buffer = Rxbuffer;
        usart3_manager.rx_buffer_size = len;
        Usart_Rx_Callback_Register(&usart3_manager, call_back_fun);
        __HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		HAL_UART_Receive_DMA(huart, Rxbuffer, len);
    }
	else if(huart->Instance == UART4)
    {
        usart4_manager.uart_handle = huart;
        usart4_manager.rx_buffer = Rxbuffer;
        usart4_manager.rx_buffer_size = len;
        Usart_Rx_Callback_Register(&usart4_manager, call_back_fun);
        __HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		HAL_UART_Receive_DMA(huart, Rxbuffer, len);
    }
	else if(huart->Instance == UART5)
    {
        usart5_manager.uart_handle = huart;
        usart5_manager.rx_buffer = Rxbuffer;
        usart5_manager.rx_buffer_size = len;
        Usart_Rx_Callback_Register(&usart5_manager, call_back_fun);
        __HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		HAL_UART_Receive_DMA(huart, Rxbuffer, len);
    }
    else if(huart->Instance == USART6)
    {
        usart6_manager.uart_handle = huart;
        usart6_manager.rx_buffer = Rxbuffer;
        usart6_manager.rx_buffer_size = len;
        Usart_Rx_Callback_Register(&usart6_manager, call_back_fun);
        __HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		HAL_UART_Receive_DMA(huart, Rxbuffer, len);
    }
    else
    {
        Error_Handler();
    }
}


/**
 * @brief   Registered user callback function
 * @param   manager: serial port handle
 * @param   fun: user callback function
 * @retval  None
 */
void Usart_Rx_Callback_Register(usart_manager_t *manager, usart_call_back fun)
{
  /* Check the parameters */
	assert_param(fun != NULL);
	assert_param(manager != NULL);
	
	manager->call_back_fun = fun;
	return;
}


/**
 * @brief   Determine if the idle interrupt is triggered
 * @param   manager: serial port handle
 * @retval  None
 */
void Uart_Receive_Handler(usart_manager_t *manager)
{
	if(__HAL_UART_GET_FLAG(manager->uart_handle,UART_FLAG_IDLE)!=RESET)
	{
		Uart_Rx_Idle_Callback(manager);
	}
}


/**
 * @brief   clear idle it flag after uart receive a frame data
 * @note    call in uart_receive_handler() function
 * @param   uart IRQHandler id
 * @retval  None
 */
static void Uart_Rx_Idle_Callback(usart_manager_t *manager)
{
    /* Check the parameters */
	assert_param(manager != NULL);
	
    /* Private variables */
	static uint16_t usart_rx_num;

    /* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(manager->uart_handle);

    /* clear DMA transfer complete flag */
	HAL_UART_DMAStop(manager->uart_handle);

    /* handle received data in idle interrupt */
	usart_rx_num = manager->rx_buffer_size - ((DMA_Stream_TypeDef*)manager->uart_handle->hdmarx->Instance)->NDTR;
	if(manager->call_back_fun != NULL)
		manager->call_back_fun(manager->rx_buffer, usart_rx_num);
	
	HAL_UART_Receive_DMA(manager->uart_handle, manager->rx_buffer, manager->rx_buffer_size);
}

/**
 * @brief 校验位函数Crc8
 * @param 传入数组
 * @param 当前数组长度
 * @return 检验值
*/
unsigned char serial_get_crc8_value(unsigned char *tem_array, unsigned char len)
{
    unsigned char crc = 0;
    unsigned char i;
    while(len--)
    {
        crc ^= *tem_array++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}





//#include <stdio.h>
//#include "stm32f4xx_hal.h"

//#pragma import(__use_no_semihosting)

//// 标准库需要的支持函数
//struct __FILE {
//	int handle;
//};

//FILE __stdout;

//// 定义_sys_exit()以避免使用半主机模式
//void _sys_exit(int x) {
//	x = x;
//}

//void _ttywrch(int ch) {
//	ch = ch;
//}

//// 重定义fputc函数
//int fputc(int ch, FILE *f) {
//	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF); // 根据使用的UART重定向
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ch, sizeof(ch));
//	return ch;
//}





#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#define SEND_BUF_SIZE 100
uint8_t Sendbuf[SEND_BUF_SIZE];

void printf_DMA(char *fmt, ...)
{
    memset(Sendbuf, 0, SEND_BUF_SIZE);  // 清空发送缓冲区
    
    va_list arg;
    va_start(arg, fmt);
    vsnprintf((char*)Sendbuf, SEND_BUF_SIZE, fmt, arg);  // 安全的格式化输出，防止缓冲区溢出
    va_end(arg);
    
    uint8_t len = strlen((char*)Sendbuf);  // 计算实际字符串长度
    if(len > 0)
	{
        HAL_UART_Transmit_DMA(&huart1, Sendbuf, len);  // 通过DMA发送字符串
    }
}

























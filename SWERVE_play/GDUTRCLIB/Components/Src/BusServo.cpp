#include "BusServo.h"

#ifdef USE_PWM_SERVO

static TIM_HandleTypeDef* pwm_htim = NULL;
static uint32_t pwm_channel;

/**
 * @brief 舵机初始化
 * @note 需先调用PWM_Servo_Init绑定定时器
 */
void PWM_Servo_Init(TIM_HandleTypeDef* htim, uint32_t channel) 
{
    pwm_htim = htim;
    pwm_channel = channel;
    HAL_TIM_PWM_Start(htim, channel);
}

/**
 * @brief 舵机控制代码
 */
void PWM_Servo_SetAngle(uint16_t angle)
{
    // 角度 0~180 映射为 500~2500us 脉宽
    uint16_t pulse = 500 + (angle * 2000) / 180;
    __HAL_TIM_SET_COMPARE(pwm_htim, pwm_channel, pulse);
}

#else

// 舵机命令定义
#define CMD_SERVO_MOVE_TIME_WRITE       0x01
#define CMD_SERVO_MOVE_STOP             0x0C
#define CMD_SERVO_ID_WRITE              0x0D
#define CMD_SERVO_ANGLE_LIMIT_WRITE     0x14
#define CMD_SERVO_POS_READ              0x1C
#define CMD_SERVO_LOAD_OR_UNLOAD_WRITE  0x1F

/**
 * @brief 构造函数
 */
 
 
BusServo::BusServo(UART_HandleTypeDef* huart, GPIO_TypeDef* dirPort, uint16_t dirPin, uint8_t id)
{
    m_uart = huart;
    m_dirPort = dirPort;
    m_dirPin = dirPin;
    m_servoId = id;
}

/**
 * @brief 析构函数
 */
 
 
BusServo::~BusServo()
{
    // 不需要特别处理
}

/**
 * @brief 初始化舵机
 */

/*
void BusServo::init()
{
    // 配置方向控制引脚
    GPIO_InitTypeDef gpioInit = {0};
    gpioInit.Pin = m_dirPin;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(m_dirPort, &gpioInit);
    
    // 初始化为接收模式
    setRxMode();
}

*/


/**
 * @brief 切换为发送模式
 */
void BusServo::setTxMode()
{
    HAL_GPIO_WritePin(m_dirPort, m_dirPin, GPIO_PIN_SET);
    // 短暂延时确保方向切换完成（根据硬件调整）
    for (volatile uint32_t i = 0; i < 100; i++);
}

/**
 * @brief 切换为接收模式
 */
void BusServo::setRxMode()
{
    HAL_GPIO_WritePin(m_dirPort, m_dirPin, GPIO_PIN_RESET);
    // 短暂延时确保方向切换完成
    for (volatile uint32_t i = 0; i < 100; i++);
}

/**
 * @brief 发送协议数据包
 */
void BusServo::sendPacket(uint8_t cmd, const uint8_t* params, uint8_t paramLen)
{
    // 限制参数长度（避免缓冲区溢出）
    if (paramLen > 6) return;

    uint8_t buffer[10] = {0};  // 协议最大帧长为10字节
    uint8_t len = paramLen + 3;  // 数据长度=参数长度+3（ID+Length+Cmd）

    // 填充帧头
    buffer[0] = 0x55;
    buffer[1] = 0x55;
    buffer[2] = m_servoId;     // 舵机ID
    buffer[3] = len;           // 数据长度字段
    buffer[4] = cmd;           // 指令值

    // 计算校验和
    uint8_t sum = m_servoId + len + cmd;
    for (uint8_t i = 0; i < paramLen; i++) 
    {
        if (params != NULL) {
            buffer[5 + i] = params[i];
            sum += params[i];
        }
    }
    buffer[5 + paramLen] = (uint8_t)~sum;  // 校验和取反
    
    // 切换为发送模式
    //setTxMode();

    // 发送完整数据包
    HAL_UART_Transmit(m_uart, buffer, 6 + paramLen, HAL_MAX_DELAY);
    
    // 返回为接收模式
   // setRxMode();
}

/**
 * @brief 舵机上电
 */
void BusServo::load()
{
    uint8_t param =  0x01;  // 参数1：上电
    sendPacket(CMD_SERVO_LOAD_OR_UNLOAD_WRITE, &param, 1);
}

/**
 * @brief 舵机掉电
 */
void BusServo::unload()
{
    uint8_t param = 0x00;  // 参数0：掉电
    sendPacket(CMD_SERVO_LOAD_OR_UNLOAD_WRITE, &param, 1);
}

/**
 * @brief 停止舵机运动
 */
void BusServo::stop()
{
    sendPacket(CMD_SERVO_MOVE_STOP, NULL, 0);  // 无参数
}

/**
 * @brief 控制舵机转动到指定角度
 */
void BusServo::moveToAngle(uint16_t angle, uint16_t time) 
{
    // 限制参数范围
    if (angle > 1000) angle = 1000;
    if (time > 30000) time = 30000;

    uint8_t params[4];
    params[0] = angle & 0xFF;        // 角度低8位
    params[1] = (angle >> 8) & 0xFF; // 角度高8位
    params[2] = time & 0xFF;         // 时间低8位
    params[3] = (time >> 8) & 0xFF;  // 时间高8位
    sendPacket(CMD_SERVO_MOVE_TIME_WRITE, params, 4);
}

/**
 * @brief 修改舵机ID
 */
void BusServo::setID(uint8_t newID)
{
    if (newID > 253) return;  // 校验ID合法性
    
    sendPacket(CMD_SERVO_ID_WRITE, &newID, 1);
    m_servoId = newID;  // 更新本地ID
}

/**
 * @brief 设置角度限位
 */
void BusServo::setAngleLimit(uint16_t minAngle, uint16_t maxAngle) 
{
    if (minAngle >= maxAngle || minAngle > 1000 || maxAngle > 1000) return;

    uint8_t params[4];
    params[0] = minAngle & 0xFF;
    params[1] = (minAngle >> 8) & 0xFF;
    params[2] = maxAngle & 0xFF;
    params[3] = (maxAngle >> 8) & 0xFF;
    sendPacket(CMD_SERVO_ANGLE_LIMIT_WRITE, params, 4);
}

/**
 * @brief 读取当前角度
 */
bool BusServo::readAngle(uint16_t* angle) 
{
    if (angle == NULL) return false;  // 校验指针有效性

    sendPacket(CMD_SERVO_POS_READ, NULL, 0);  // 发送读指令

    uint8_t buf[8];  // 接收缓冲区
    if (HAL_UART_Receive(m_uart, buf, 8, 500) != HAL_OK)
    {
        return false;  // 接收超时
    }

    // 验证帧头
    if (buf[0] != 0x55 || buf[1] != 0x55) return false;

    // 验证ID和指令
    if (buf[2] != m_servoId || buf[4] != CMD_SERVO_POS_READ) return false;

    // 验证校验和
    uint8_t sum = buf[2] + buf[3] + buf[4] + buf[5] + buf[6];
    if (buf[7] != (uint8_t)~sum) return false;

    // 提取角度值
    *angle = (uint16_t)(buf[5] | (buf[6] << 8));
    return true;
}

#endif
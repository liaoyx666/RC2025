#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>



#ifdef USE_PWM_SERVO
void PWM_Servo_Init(TIM_HandleTypeDef* htim, uint32_t channel);
void PWM_Servo_SetAngle(uint16_t angle);

#else
#ifdef __cplusplus
class BusServo {
private:
    UART_HandleTypeDef* m_uart;    // 串口句柄
    GPIO_TypeDef* m_dirPort;       // 方向控制引脚端口
    uint16_t m_dirPin;             // 方向控制引脚号
    uint8_t m_servoId;             // 舵机ID
    
    // 私有方法
    void setTxMode();              // 切换为发送模式
    void setRxMode();              // 切换为接收模式
    void sendPacket(uint8_t cmd, const uint8_t* params, uint8_t paramLen);
    
public:
    // 构造函数和析构函数
    BusServo(UART_HandleTypeDef* huart, GPIO_TypeDef* dirPort, uint16_t dirPin, uint8_t id);
    ~BusServo();
    
    // 公共方法
    //void init();                   // 初始化舵机
    void load();                   // 舵机上电
    void unload();                 // 舵机掉电
    void stop();                   // 停止舵机运动
    void moveToAngle(uint16_t angle, uint16_t time); // 控制舵机转动到指定角度
    void setID(uint8_t newID);     // 修改舵机ID
    void setAngleLimit(uint16_t minAngle, uint16_t maxAngle); // 设置角度限位
    bool readAngle(uint16_t* angle); // 读取当前角度
};

#endif


#endif


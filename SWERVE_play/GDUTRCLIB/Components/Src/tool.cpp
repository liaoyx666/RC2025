#include "tool.h"


/***********************
 * 类型转换辅助函数
***********************/
//16位int数据填充
void _tool_buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) 
{
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

//16位unsigned int数据填充
void _tool_buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) 
{
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

//32位int数据填充
void _tool_buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

//32位unsigned int数据填充
void _tool_buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) 
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

/**
 * @brief 浮点转换函数,16位float数据填充
 * @param 数据缓存数组
 * @param 数据
 * @param 数量级
 * @param unsign_flag 是否使用unsigned,使用置1，否则置0
*/
void _tool_buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index, bool unsign_flag) 
{
    if(unsign_flag)
        _tool_buffer_append_int16(buffer, (int16_t)(number * scale), index);
    else
        _tool_buffer_append_uint16(buffer, (uint16_t)(number * scale), index);
}

/**
 * @brief 浮点转换函数,32位float数据填充
 * @param 数据缓存数组
 * @param 数据
 * @param 数量级
 * @param unsign_flag 是否使用unsigned,使用置1，否则置0
*/
void _tool_buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index, bool unsign_flag)
{
    if(unsign_flag)
        _tool_buffer_append_uint32(buffer, (uint32_t)(number * scale), index);
    else
        _tool_buffer_append_int32(buffer, (int32_t)(number * scale), index);
}


/**
 * @brief 16位int数据获取
 * @param 数据缓存数组
*/
int16_t _tool_buffer_get_int16(const uint8_t *buffer, int32_t *index)
{
    int16_t res = ((uint16_t)buffer[*index]) << 8 |
                ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}


/**
 * @brief 16位uint数据获取
 * @param 数据缓存数组
*/
uint16_t _tool_buffer_get_uint16(const uint8_t *buffer, int32_t *index)
{
    uint16_t res = ((uint16_t)buffer[*index]) << 8 |
                ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}


/**
 * @brief 32位int数据获取
 * @param 数据缓存数组
*/
int32_t _tool_buffer_get_int32(const uint8_t *buffer, int32_t *index)
{
    int32_t res = ((uint32_t)buffer[*index]) << 24      |
                ((uint32_t)buffer[*index + 1]) << 16  |
                ((uint32_t)buffer[*index + 2]) << 8   |
                ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}


/**
 * @brief 16位uint数据获取
 * @param 数据缓存数组
*/
uint32_t _tool_buffer_get_uint32(const uint8_t *buffer, int32_t *index)
{
    uint32_t res = ((uint32_t)buffer[*index]) << 24     |
                ((uint32_t)buffer[*index + 1]) << 16 |
                ((uint32_t)buffer[*index + 2]) << 8  |
                ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}


/**
 * @brief 浮点转换函数,16位float数据获取
 * @param 数据缓存数组
 * @param 数据
 * @param 数量级
 * @param unsign_flag 是否使用unsigned,使用置1，否则置0
*/
float _tool_buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag)
{
    if(unsign_flag)
        return (float)_tool_buffer_get_uint16(buffer, index) / scale;
    else
        return (float)_tool_buffer_get_int16(buffer, index) / scale;
}


/**
 * @brief 浮点转换函数,32位float数据填充
 * @param 数据缓存数组
 * @param 数据
 * @param 数量级
 * @param unsign_flag 是否使用unsigned,使用置1，否则置0
*/
float _tool_buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag)
{
    if(unsign_flag)
        return (float)_tool_buffer_get_uint32(buffer, index) / scale;
    else
        return (float)_tool_buffer_get_int32(buffer, index) / scale;
}

float DM_fmaxf(float a,float b)//a，b取最大
{
    return a>=b?a:b;
}

float DM_fminf(float a,float b)//a，b取最小
{
    return a<=b?a:b;
}

/***浮点型转整形***
入口参数：浮点数据、该数据最小值、该数据最大值、位数
*****************/
int DM_float_to_uint(float x1,float x1_min,float x1_max,int bits)
{
    float span = x1_max-x1_min;
    float offset = x1_min;
    return (int)((x1-offset)*((float)((1<<bits)-1))/span);
}

//整型转浮点型
//根据给定的范围和位数，将无符号整数转换为浮点
float DM_uint_to_float(int x1_int,float x1_min,float x1_max,int bits)
{
    float span=x1_max-x1_min;
    float offset=x1_min;
    return ((float)x1_int)*span/((float)((1<<bits)-1)) + offset;
}

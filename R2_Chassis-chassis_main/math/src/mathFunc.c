#include "mathFunc.h"
#include "main.h"

/**
 * @brief 将p1和p2进行互换，在地址上操作
 * 
 * @param p1 
 * @param p2 
 */
void ChangeDataByte(uint8_t *p1, uint8_t *p2)
{
    uint8_t t;
    t = *p1;
    *p1 = *p2;
    *p2 = t;
}

/**
 * @brief Get the s32 from buffer object 将4个8位合成32位 同时改变index
 * 
 * @param buffer 
 * @param index 
 * @return int32_t 
 */
int32_t get_s32_from_buffer(const uint8_t *buffer, int32_t *index)
{
    int32_t res = (((uint32_t)buffer[*index]) << 24) |
                  (((uint32_t)buffer[*index + 1]) << 16) |
                  (((uint32_t)buffer[*index + 2]) << 8) |
                  (((uint32_t)buffer[*index + 3]));
    *index += 4;
    return res;
}

/**
 * @brief Get the s16 from buffer object 将两个8位合成16位 同时改变index
 * 
 * @param buffer 
 * @param index 
 * @return int16_t 
 */
int16_t get_s16_from_buffer(const uint8_t *buffer, int32_t *index)
{
    int16_t res = (((uint32_t)buffer[*index]) << 8) |
                  (((uint32_t)buffer[*index + 1]));
    *index += 2;
    return res;
}

/**
 * @brief 将32位转为float 同时改变index 除以scale
 * 
 * @param buffer 
 * @param scale 
 * @param index 
 * @return float 
 */
float buffer_32_to_float(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)get_s32_from_buffer(buffer, index) / scale;
}

/**
 * @brief 将16位转为float
 * 
 * @param buffer 
 * @param scale 
 * @param index 
 * @return float 
 */
float buffer_16_to_float(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)get_s16_from_buffer(buffer, index) / scale;
}

/**
 * @brief 将一个32位转为4个8位
 * 
 * @param buffer 
 * @param source 
 * @param index 
 */
void buffer_append_int32(uint8_t *buffer, int32_t source, int32_t *index)
{
    buffer[(*index)++] = source >> 24;
    buffer[(*index)++] = source >> 16;
    buffer[(*index)++] = source >> 8;
    buffer[(*index)++] = source;
}
/**
 * @brief 将一个16位转为2个8位
 * 
 * @param buffer 
 * @param source 
 * @param index 
 */
void buffer_append_int16(uint8_t *buffer, int16_t source, int32_t *index)
{
    buffer[(*index)++] = source;
    buffer[(*index)++] = source >> 8;//小端模式
}
/**
 * @brief 将两个float转化为double
 * 
 * @param n1 
 * @param n2 
 * @return double 
 */
double cvtFloat2Double(float n1, float n2)
{
	struct {float n1;float n2;} s;
	s.n1 = n1;
	s.n2 = n2;
	return *((double*)&s);
}

/**
 * @brief 用于关节电机的数据转化
 * 
 * @param x_int 
 * @param x_min 
 * @param x_max 
 * @param bits 
 * @return float 
 */
float uint2float(int x_int, float x_min, float x_max, int bits)
{
	float span	 = x_max - x_min;
	float offset = x_min;
	return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

u16 float2uint(float x, float x_min, float x_max, uint8_t bits)
{
	float span	 = x_max - x_min;
	float offset = x_min;

	return (u16) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}
float Lerp(float start,float end,float t)
{
    if(t > 1)
        t = 1;
    else if(t < 0)
        t = 0;
    return (end-start) * t + start;
}

float N2DEG(float N)
{
	return N * 360;
}
/**
 * @brief 旋转度 将诸如贝塞尔曲线上取点进行旋转变换
 * 
 * @param x 结果 x
 * @param y 结果 y
 * @param x0 
 * @param y0 
 * @param a 旋转度 度为单位
 */
void Rotate(float* x, float* y, float x0, float y0, float a)
{
	float x1 = *x;
	float y1 = *y;
    float rad = a * PI / 180.f;
	*x		 = (x1 - x0) * cosf(rad) - (y1 - y0) * sinf(rad) + x0;
	*y		 = (y1 - y0) * cosf(rad) + (x1 - x0) * sinf(rad) + y0;
}

float DEG2RAD(float angle)
{
    return angle / 180.f * PI;
}
float RAD2DEG(float angle)
{
    return angle / PI * 180.f;
}
/**
 * @brief 将字节数组转换为16位整数
 * @param buff 字节数组
 * @param i 读取的起始索引
 * @return 转换后的16位整数
 */
s16 MSG_Byte2Int16(uint8_t *buff, uint8_t i) {
  s16 data = (s16)(buff[i + 1] << 8 | buff[i]);
  return data;
}
int MSG_Byte2Int32(uint8_t *buff, uint8_t i) {
    int data =
      (int)(buff[i + 3] << 24 | buff[i + 2] << 16 | buff[i + 1] << 8 | buff[i]);
  return data;
}
/**
 * @brief 将16位整数转换为字节数组
 * @param data 待转换的16位整数
 * @param buff 字节数组
 * @param i 写入的起始索引
 */
void MSG_Int162Byte(s16 data, uint8_t *buff, uint8_t i) {
  buff[i] = (uint8_t)(data & 0xff);
  buff[i + 1] = (uint8_t)(data >> 8);
}
void MSG_Int322Byte(int data, uint8_t *buff, uint8_t i)
{
  buff[i] = (uint8_t)(data & 0xff);
  buff[i + 1] = (uint8_t)((int)data >> 8 & 0xff);
  buff[i + 2] = (uint8_t)((int)data >> 16 & 0xff);
  buff[i + 3] = (uint8_t)((int)data >> 24);
}

#ifndef MATH_SUPPORT_H
#define MATH_SUPPORT_H

#include "struct_typedef.h"

#include "arm_math.h"


extern const float MATH_RPM_TO_RADPS;
extern const float MATH_DEG_TO_RAD;
extern const float MATH_CELSIUS_TO_KELVIN;

template<typename Type>
Type Basic_Math_Abs(Type x)
{
    return ((x > 0) ? x : -x);
}


/**
  * @brief          int 转 fp32 
  * @param[in]      x_int 要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */

fp32 uint_to_float(uint16_t x_int, fp32 x_min, fp32 x_max, uint8_t bits);

/**
  * @brief          fp32 转 int 
  * @param[in]      x     要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */
uint16_t float_to_uint_f(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits);

uint16_t float_to_uint_i(fp32 x, uint16_t x_min, uint16_t x_max, uint8_t bits);

fp32 motor_max_min(fp32 proto,fp32 max,fp32 min);

void LimitMax(fp32 input,fp32 max);

uint16_t Math_Endian_Reverse_16(void *Source, void *Destination);

#endif

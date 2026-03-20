#include "math_support.h"

// rad换算到deg
const float MATH_RPM_TO_RADPS = 2.0f * PI / 60.0f;
// deg换算到rad
const float MATH_DEG_TO_RAD = PI / 180.0f;
// 摄氏度换算到开氏度
const float MATH_CELSIUS_TO_KELVIN = 273.15f;

/**
  * @brief          int 转 fp32 
  * @param[in]      x_int 要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */
fp32 uint_to_float(uint16_t x_int, fp32 x_min, fp32 x_max, uint8_t bits){
	fp32 span=x_max-x_min;
	fp32 offset = x_min;
	return ((fp32)x_int)*span/((fp32)((1<<bits)-1)) + offset;
}

/**
  * @brief          fp32 转 int 
  * @param[in]      x     要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */
uint16_t float_to_uint_f(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits){
	fp32 span=x_max-x_min;
	fp32 offset = x_min;
	return (uint16_t) ((x-offset)*((fp32)((1<<bits)-1))/span) ;
}

uint16_t float_to_uint_i(fp32 x, uint16_t x_min, uint16_t x_max, uint8_t bits){
	fp32 span=x_max-x_min;
	fp32 offset = x_min;
	return (uint16_t) ((x-offset)*((fp32)((1<<bits)-1))/span) ;
}

fp32 motor_max_min(fp32 proto,fp32 max,fp32 min){
	if(proto>max)return max;
	else if(proto<min) return min;
	return proto;
} 


void LimitMax(fp32 input,fp32 max)
{   
    {                          
        if (input > max)       
        {                      
            input = max;       
        }                      
        else if (input < -max) 
        {                      
            input = -max;      
        }                      
    }
}


uint16_t Math_Endian_Reverse_16(void *Source, void *Destination)
{
    uint8_t *tmp_address_8;
    uint16_t tmp_value_16;
    tmp_address_8 = (uint8_t *) Source;
    tmp_value_16 = tmp_address_8[0] << 8 | tmp_address_8[1];

    if (Destination != nullptr)
    {
        uint8_t *tmp_source, *tmp_destination;
        tmp_source = (uint8_t *) Source;
        tmp_destination = (uint8_t *) Destination;
        tmp_destination[0] = tmp_source[1];
        tmp_destination[1] = tmp_source[0];
    }

    return (tmp_value_16);
}

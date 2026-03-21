//
// Created by 20159 on 2026/3/13.
//

#include "motor_YS.h"
#include "crc_ccitt.h"
#include "math_support.h"
#include "string.h"

//把数据放在0x24000000之后因为DMA原因
__attribute__((section (".AXI_SRAM")))  uint8_t motor_YS_send_Temp_buffer[17];
__attribute__((section (".AXI_SRAM")))  uint8_t motor_YS_recv_Temp_buffer[16];

/**
 * @brief 宇树电机的初始化
 * @param huart 串口编号
 * @param __id   电机ID
 * @param __control_mode 角度的控制还是弧度控制  
 * @return void
 */
void Class_Motor_YS::Init(UART_HandleTypeDef *huart,uint16_t __id,const Enum_Motor_YS_Mode &__control_mode)
{
    UART = huart;
    id = __id;
    Motor_YS_Status = Motor_YS_Status_DISABLE;
    init_Pos = 0.0f;
    Gearbox_Rate = 6.33;
    control_mode = __control_mode;
}

/**
 * @brief 宇树电机数据处理
 * @return void
 */
void Class_Motor_YS::send_data(){
    uint16_t crc;
    uint32_t YS_T,YS_W,YS_Angle_Pos,YS_Kp,YS_Kd;    
    motor_YS_send_Temp_buffer[0] = 0xFE;
    motor_YS_send_Temp_buffer[1] = 0xEE;
    motor_YS_send_Temp_buffer[2] = id | Motor_YS_Status<<4;
    YS_T = T * 256;
    YS_W = W * 128 /PI*Gearbox_Rate;
    if(control_mode == Motor_YS_Pos_control)
        YS_Angle_Pos = (Pos+init_Pos) * 16384 * Gearbox_Rate /PI;
    else 
        YS_Angle_Pos = (Angle+init_Angle) *MATH_DEG_TO_RAD* 16384 * Gearbox_Rate /PI;

    YS_Kp = Kp * 1280;
    YS_Kd = Kd * 1280;
    YS_T = motor_max_min(YS_T,127.99f,-127.99f);
    YS_W = motor_max_min(YS_W,804.0f,-804.0f);
    YS_Angle_Pos = motor_max_min(YS_Angle_Pos,411774.0f,-411774.0f);
    YS_Kp = motor_max_min(YS_Kp,25.599f,-25.599f);
    YS_Kd = motor_max_min(YS_Kd,25.599f,-25.599f);
    motor_YS_send_Temp_buffer[3] = (YS_T) & 0xFF;
    motor_YS_send_Temp_buffer[4] = (YS_T>>8) & 0xFF;
    motor_YS_send_Temp_buffer[5] = (YS_W) & 0xFF;
    motor_YS_send_Temp_buffer[6] = (YS_W>>8) & 0xFF;
    motor_YS_send_Temp_buffer[7] = (YS_Angle_Pos) & 0xFF;
    motor_YS_send_Temp_buffer[8] = (YS_Angle_Pos>>8) & 0xFF;
    motor_YS_send_Temp_buffer[9] = (YS_Angle_Pos>>16) & 0xFF;
    motor_YS_send_Temp_buffer[10] = (YS_Angle_Pos>>24) & 0xFF;
    motor_YS_send_Temp_buffer[11] = (YS_Kp) & 0xFF;
    motor_YS_send_Temp_buffer[12] = (YS_Kp>>8) & 0xFF;
    motor_YS_send_Temp_buffer[13] = (YS_Kd) & 0xFF;
    motor_YS_send_Temp_buffer[14] = (YS_Kd>>8) & 0xFF;
    
    crc = crc_ccitt(motor_YS_send_Temp_buffer, sizeof(motor_YS_send_Temp_buffer)-2);

    motor_YS_send_Temp_buffer[15] = (crc) & 0xFF;
    motor_YS_send_Temp_buffer[16] = (crc>>8) & 0xFF;

}

/**
 * @brief 宇树电机数据发送
 * @return void
 */
void Class_Motor_YS::UART_send(){
    send_data();
    UART_Transmit_Data(UART,motor_YS_send_Temp_buffer);
}

/**
 * @brief 宇树电机使能
 * @return void
 */
void Class_Motor_YS::enable(){
    Motor_YS_Status = Motor_YS_Status_ENABLE;    
    UART_send();
}

/**
 * @brief 宇树电机失能
 * @return void
 */
void Class_Motor_YS::lose(){
    Motor_YS_Status = Motor_YS_Status_DISABLE;    
    UART_send();
}

/**
 * @brief 宇树电机零点设置(软件办法)
 * @return void
 */
void Class_Motor_YS::zero(){
    if(control_mode == Motor_YS_Pos_control)Pos = recv.Now_Pos;
    else Angle = recv.Now_Angle;
}

/**
 * @brief 宇树电机的返回数据处理
 * @param data 电机返回数据数组指针
 * @return void
 */
void Class_Motor_YS::UART_recv(uint8_t *data){
    int16_t YS_T,YS_W,YS_Angle_Pos,YS_Kp,YS_Kd;
    
    memcpy(motor_YS_recv_Temp_buffer,data,16);
    
    mode = motor_YS_recv_Temp_buffer[2]>>4 & 0xF;
    YS_T = motor_YS_recv_Temp_buffer[4]<<8 | motor_YS_recv_Temp_buffer[3];
    recv.Now_T  = YS_T;
    recv.Now_T /= 256;
    YS_W = motor_YS_recv_Temp_buffer[6]<<8 | motor_YS_recv_Temp_buffer[5];
    recv.Now_W = YS_W;
    recv.Now_W  = (recv.Now_W   * 6.28319 )/(256*6.33);
	YS_Angle_Pos = motor_YS_recv_Temp_buffer[10]<<24 | motor_YS_recv_Temp_buffer[9]<<16 | 
                    motor_YS_recv_Temp_buffer[8]<<8 | motor_YS_recv_Temp_buffer[7];
    recv.Now_Pos=YS_Angle_Pos;
	recv.Now_Pos= (recv.Now_Pos * 6.28319 )/(32768*6.33);
    recv.Now_Angle = recv.Now_Pos/MATH_RPM_TO_RADPS;
    int8_t Temp = motor_YS_recv_Temp_buffer[11] & 0xFF;
    recv.Now_Temperature = Temp;
    recv.MError = motor_YS_recv_Temp_buffer[12] & 0x7;
}


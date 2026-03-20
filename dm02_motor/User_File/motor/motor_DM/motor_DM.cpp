//
// Created by 20159 on 2026/3/13.
//

#include "motor_DM.h"
#include "math_support.h"

//MIT 位置 速度 数据发送
static uint8_t motor_send_DM_Data[8];
//失能
static uint8_t motor_send_DM_Data_lose[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
//使能
static uint8_t motor_send_DM_Data_enable[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
//零点设置
static uint8_t motor_send_DM_Data_zero[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};


/**
 * @brief 达妙电机的初始化
 * @param hfdcan CAN编号
 * @param __id   CAN_ID
 * @param __model 电机的型号 MOTOR_DJ_M2006=0 & MOTOR_DJ_M3508 & MOTOR_DJ_M6020
 * @param __control_mode 角度的控制还是弧度控制  
 * @return void
 */
void Class_Motor_DM::Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,const motor_DM_Model &__model,
            const Enum_Motor_DM_Mode &__control_mode)
{
    FDcan = hfdcan;
    CAN_id = __id;
    model = __model;
    control_mode = __control_mode;
    switch (model)
    {
        case MOTOR_DM_J10010L:
            Pos_Max=12.57f;
            W_Max = 50.00f;
            T_Max = 100.0f;
        break;
        case MOTOR_DM_J4310:
            Pos_Max=12.57f;
            W_Max = 30.00f;
            T_Max = 30.0f;
        break;
    
    }
    enable();
}

/**
 * @brief 达妙电机的使能
 * @return void
 */
void Class_Motor_DM::enable(){
    fdcan_send_data_stand(FDcan,CAN_id,motor_send_DM_Data_enable,8);
}

/**
 * @brief 达妙电机的失能
 * @return void
 */
void Class_Motor_DM::lose(){
    fdcan_send_data_stand(FDcan,CAN_id,motor_send_DM_Data_lose ,8);
}

/**
 * @brief 达妙电机的零点设置
 * @return void
 */
void Class_Motor_DM::zero(){
    fdcan_send_data_stand(FDcan,CAN_id,motor_send_DM_Data_zero,8);
}

/**
 * @brief 达妙电机的数据发送
 * @return void
 */
void Class_Motor_DM::can_send(){
    static uint16_t motor[5];
    if(control_mode == Motor_DM_Angle_control) Pos=Angle*MATH_DEG_TO_RAD;
    motor[3]=float_to_uint_f(Kp,0.0f,500.0f,12);
	motor[4]=float_to_uint_f(Kd,0.0f,5.0f,12);
	motor[0]=float_to_uint_f(Pos,-Pos_Max,Pos_Max,16);
	motor[1]=float_to_uint_f(W,-W_Max,W_Max,12);
	motor[2]=float_to_uint_f(T,-T_Max,T_Max,12);

    motor_send_DM_Data[0]=motor[0]>>8;
	motor_send_DM_Data[1]=motor[0];
	motor_send_DM_Data[2]=motor[1]>>4;
	motor_send_DM_Data[3]=(motor[1]&0xF)<<4|(motor[3]>>8);
	motor_send_DM_Data[4]=motor[3];
	motor_send_DM_Data[5]=motor[4]>>4;
	motor_send_DM_Data[6]=(motor[4]&0xF)<<4|(motor[2]>>8);
	motor_send_DM_Data[7]=motor[2];

    fdcan_send_data_stand(FDcan,CAN_id,motor_send_DM_Data,8);
}


/**
 * @brief 达妙电机的数据接收
 * @param *data 接收数据数组指针
 * @return void
 */
void Class_Motor_DM::can_recv(uint8_t *data){
    recv.mos_Temperature 	= (fp32)data[6];
	recv.motor_Temperature  = (fp32)data[7];
	recv.MError  = data[0]>>4;
	
	recv.Now_Pos = uint_to_float((data[1]<<8|data[2]),-Pos_Max,Pos_Max,16);
	recv.Now_W  = uint_to_float((data[3]<<4|data[4]>>4),-W_Max,W_Max,12);
	recv.Now_T  =uint_to_float((data[4]&0x0F)<<8|data[5],-T_Max,T_Max,12);

    recv.Now_Angle = recv.Now_Pos*MATH_RPM_TO_RADPS;
    Motor_DM_Status = Motor_DM_Status_ENABLE;
}

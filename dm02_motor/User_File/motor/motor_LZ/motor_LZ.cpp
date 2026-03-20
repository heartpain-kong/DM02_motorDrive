//
// Created by 20159 on 2026/3/13.
//

#include "motor_LZ.h"
#include "math_support.h"

//电机数据发送
uint8_t motor_send_LZ_Data[8];
//失能 使能 的数据
uint8_t motor_send_LZ_Data_lose_enable[8]={00,00,00,00,00,00,00,00};
//主动上报模式
uint8_t motor_send_LZ_Data_active_recv[8]={01,02,03,04,05,06,00,00};
//CAN_ID的修改 零点设置
uint8_t motor_send_LZ_Data_zero_set_CAN_ID[8] = {01,00,00,00,00,00,00,00};


/**
 * @brief 灵足电机的初始化
 * @param hfdcan CAN编号
 * @param __id   CAN_ID
 * @param __model 电机的型号 
 * @param __control_mode 角度的控制还是弧度控制  
 * @return void
 */
void Class_Motor_LZ::Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,const motor_LZ_Model &__model,
        const Enum_Motor_LZ_Mode &__control_mode)
{
    FDcan = hfdcan;
    CAN_id = __id;
    model = __model;
    control_mode = __control_mode;
    switch (model)
    {
        case MOTOR_LZ_02:
            Pos_Max=12.57f;
            W_Max = 44.00f;
            T_Max = 17.0f;
        break;
        case MOTOR_LZ_05:
            Pos_Max=12.57f;
            W_Max = 50.00f;
            T_Max = 5.5f;
        break;
    
    }

}
/**
 * @brief 灵足电机扩展帧的处理
 * @param  灵足把扩展帧分为5+16+8三段中的16
 * @return 灵足电机扩展帧
 */
uint32_t Class_Motor_LZ::send_data(uint32_t Id_data){
    uint32_t motor_LZ_send_box=0;
    motor_LZ_send_box = ((uint32_t)mode)<<24 | Id_data<<8 | CAN_id;
	return motor_LZ_send_box;
}

/**
 * @brief 灵足电机的使能
 * @return void
 */
void Class_Motor_LZ::enable(){
    mode = CANCOM_MOTOR_IN;
    Motor_LZ_Status = Motor_LZ_Status_ENABLE;
    fdcan_send_data_Exten(FDcan,send_data(motor_LZ_user_id),motor_send_LZ_Data_lose_enable,8);
}

/**
 * @brief 灵足电机的失能
 * @return void
 */
void Class_Motor_LZ::lose(){
    mode = CANCOM_MOTOR_FEEDBACK;
    Motor_LZ_Status = Motor_LZ_Status_DISABLE;
	fdcan_send_data_Exten(FDcan,send_data(motor_LZ_user_id),motor_send_LZ_Data_lose_enable,8);
}

/**
 * @brief 灵足电机的主动上报模式
 * @param F_CMD 1为开启 0为关闭
 * @return void
 */
void Class_Motor_LZ::active_recv(uint8_t F_CMD){
    mode = CANCOM_MODE_ACTIVE_RECV;
    motor_send_LZ_Data_active_recv[6]=(F_CMD==1);
	fdcan_send_data_Exten(FDcan,send_data(motor_LZ_user_id),motor_send_LZ_Data_active_recv,8);
}

/**
 * @brief 灵足电机的零点设置
 * @return void
 */
void Class_Motor_LZ::zero(){
    lose();
    mode = CANCOM_MOTOR_ZERO;
	fdcan_send_data_Exten(FDcan,send_data(motor_LZ_user_id),motor_send_LZ_Data_zero_set_CAN_ID,8);
    enable();
}

/**
 * @brief 灵足电机的主动上报模式
 * @param set_id 需要修改的id 此函数
 * @return void
 */
void Class_Motor_LZ::motor_set_CAN_ID(uint8_t set_id){
    lose();
    mode = CANCOM_MOTOR_ZERO;
	fdcan_send_data_Exten(FDcan,send_data(motor_LZ_user_id<<8 | set_id),motor_send_LZ_Data_zero_set_CAN_ID,8);
    CAN_id=set_id;
}

/**
 * @brief 灵足电机的返回数据处理
 * @param Data 返回的扩展帧ID
 * @param *data 返回数据数组指针
 * @return void
 */
void Class_Motor_LZ::can_recv(uint32_t Data , uint8_t *data){
    recv.Now_Pos = uint_to_float((data[0]<<8|data[1]),-Pos_Max,Pos_Max,16);
    recv.Now_Angle = recv.Now_Pos * MATH_RPM_TO_RADPS;
    recv.Now_W =  uint_to_float((data[2]<<8|data[3]),-W_Max,W_Max,16);
    recv.Now_T =  uint_to_float((data[4]<<8|data[5]),-T_Max,T_Max,16);
    recv.MError= Data>>16&0x3F;
	recv.mode  = Data>>22&0x03;
    recv.Now_Temperature = (fp32)(data[6]<<8|data[7])/10.0f;
    Motor_LZ_Status = Motor_LZ_Status_ENABLE;
}

/**
 * @brief 灵足电机的数据发送
 * @return void
 */
void Class_Motor_LZ::can_send(){
    if(Motor_LZ_Status == Motor_LZ_Status_ENABLE){
        mode = CANCOM_MOTOR_CTRL;

        if(control_mode == Motor_LZ_Angle_control) Pos = Angle * MATH_DEG_TO_RAD;
        
        Pos = motor_max_min(Pos,Pos_Max,-Pos_Max);
        T = motor_max_min(T,T_Max,-T_Max);
        W = motor_max_min(W,W_Max,-W_Max);
	    Kd = motor_max_min(Kd,5.0f,0.0f);
	    Kp = motor_max_min(Kp,500.0f,0.0f);

        uint16_t motor_lz_send[5]; //angle W T kp kd
        
        motor_lz_send[0] = (uint16_t)((Pos+Pos_Max)*65535/(Pos_Max+Pos_Max));
    	motor_lz_send[1] = (uint16_t)((W+W_Max)*65535/(W_Max+W_Max));
        motor_lz_send[2] = (uint16_t)((T+T_Max)*65535/(T_Max+T_Max));
    	motor_lz_send[3] = (uint16_t)((Kp*65535/500.0f));
	    motor_lz_send[4] = (uint16_t)((Kd*65535/5.00f));

        motor_send_LZ_Data[0]=motor_lz_send[0]>>8;
    	motor_send_LZ_Data[1]=motor_lz_send[0];
	
	    motor_send_LZ_Data[2]=motor_lz_send[1]>>8;
	    motor_send_LZ_Data[3]=motor_lz_send[1];
	
	    motor_send_LZ_Data[4]=motor_lz_send[3]>>8;
    	motor_send_LZ_Data[5]=motor_lz_send[3];	
	
	    motor_send_LZ_Data[6]=motor_lz_send[4]>>8;
	    motor_send_LZ_Data[7]=motor_lz_send[4];
        
	    fdcan_send_data_Exten(FDcan,send_data(motor_lz_send[2]),motor_send_LZ_Data,8);
    }
}

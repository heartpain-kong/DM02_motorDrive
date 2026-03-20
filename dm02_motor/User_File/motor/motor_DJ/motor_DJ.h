//
// Created by 20159 on 2026/3/13.
//

#ifndef MOTOR_DJ_H
#define MOTOR_DJ_H

#include "struct_typedef.h"
#include "bsp_fdcan.h"
#include "alg_pid.h"

//电机的状态
enum Enum_Motor_DJ_Status
{
    Motor_DJ_Status_DISABLE = 0,
    Motor_Dj_Status_ENABLE,
};

//电机控制模式
enum motor_DJ_Control_mode
{
    motor_DJ_Angle  = 0,
	motor_DJ_speed = 1,
	motor_DJ_T     = 2,
};

//电机的型号
enum motor_DJ_Model{
    MOTOR_DJ_M2006=0,
	MOTOR_DJ_M3508,
	MOTOR_DJ_M6020,

};

//dji电机的CAN_ID
enum motorDJ_CAN_ID{
	CAN_M1_M4_ID = 0x200,
    CAN_M1_ID = 0x201,
    CAN_M2_ID = 0x202,
    CAN_M3_ID = 0x203,
    CAN_M4_ID = 0x204,
	
	CAN_M5_M8_ID = 0x1FF,
    CAN_M5_ID = 0x205,
    CAN_M6_ID = 0x206,
    CAN_M7_ID = 0x207,
	CAN_M8_ID = 0x208,
	
	CAN_M9_MA_ID = 0x2FF,
	CAN_M9_ID = 0x209,
    CAN_MA_ID = 0x20A,
    CAN_MB_ID = 0x20B,
};

//返回数据的结构体
struct Struct_recv_motor_DJ
{
    fp32 Now_pos;
    fp32 Now_T;
    fp32 Now_W;
    fp32 Temperature;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};

//输出的结构体
struct Struct_send_motor_DJ
{
	fp32 Pos;
	fp32 T;
	fp32 W;	
};

//电机的控制结构体里面包含了电机的发送和电机的返回
struct motor_dj_control
{
	Struct_send_motor_DJ send;
	Struct_recv_motor_DJ recv;
};

/**
 * @brief dji电机的发送
 * @return void
 */
void motor_DJ_can_send();

//创建dji电机类
class Class_Motor_DJ{

	//公共
	public:

	//位置环
    Class_PID PID_Angle;

	//速度环
    Class_PID PID_speed;

	//初始化
	void Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,motor_DJ_Model __model,
        const motor_DJ_Control_mode &__control_mode);

	//dji电机的数据的存储
    void can_send_Data();

	//运用pid处理数据
    void PID_Calculate_Data();

	//电机数据的接收处理
	void can_recv(uint8_t *data);
	
	//以下为数据保护部分代码 外部访问内部变量需要运用Get和Set函数
	inline Enum_Motor_DJ_Status Get_Status();

	inline void Set_Status(const Enum_Motor_DJ_Status &__Status);
	
	inline void Set_Pos(const fp32 &__pos);

	inline fp32 Get_Pos() const;

	inline void Set_W(const fp32 &__w);

	inline fp32 Get_W() const;

	inline void Set_T(const fp32 &__T);

	inline fp32 Get_Now_pos() const;

	inline fp32 Get_Now_T() const;

	inline fp32 Get_Now_W() const;

    inline fp32 Get_Temperature() const;
	
	//内部
	protected:
	//CAN的id
	uint16_t CAN_id;
	//型号
	motor_DJ_Model model;
	//CAN几
	FDCAN_HandleTypeDef *FDcan;
	
	//减速比
    float Gearbox_Rate;

    // 一圈编码器刻度
    const uint16_t ENCODER_NUM_PER_ROUND = 8192;

    // 扭矩电流常数, 减速前
    float CURRENT_TO_TORQUE = 0.0f;
    // 电流到输出的转化系数
    float CURRENT_TO_OUT = 0.0f;
    // 最大输出刻度
    float OUT_MAX = 0.0f;    

    // 编码器偏移
    int32_t Encoder_Offset;

	//控制模式
    motor_DJ_Control_mode Motor_DJ_Control_mode = motor_DJ_speed;

	//状态
	Enum_Motor_DJ_Status Motor_DJ_Status = Motor_DJ_Status_DISABLE;

	//返回数据
	Struct_recv_motor_DJ recv;
	//发送角度
	fp32 Pos;
	//发送力
	fp32 T;
	//发送位置
	fp32 W;	
	//输入电流
    fp32 out;
};


inline Enum_Motor_DJ_Status Class_Motor_DJ::Get_Status()
{
	return (Motor_DJ_Status);
}

inline void Class_Motor_DJ::Set_Status(const Enum_Motor_DJ_Status &__Status){
	Motor_DJ_Status = __Status;
}

inline void Class_Motor_DJ::Set_Pos(const fp32 &__pos){
	Pos = __pos;
}

inline fp32 Class_Motor_DJ::Get_Pos() const{
	return (Pos);
}

inline void Class_Motor_DJ::Set_W(const fp32 &__w){
	W=__w;
}

inline fp32 Class_Motor_DJ::Get_W() const{
	return (W);
}

inline void Class_Motor_DJ::Set_T(const fp32 &__T){
	T=__T;
}

inline fp32 Class_Motor_DJ::Get_Now_pos()const{
	return (recv.Now_pos);
}	

inline fp32 Class_Motor_DJ::Get_Now_W()const{
	return (recv.Now_W);
}

inline fp32 Class_Motor_DJ::Get_Now_T()const{
	return (recv.Now_T);
}
	
inline fp32 Class_Motor_DJ::Get_Temperature() const{
	return (recv.Temperature);
}



#endif //DM02_MOTOR_DJ_H    
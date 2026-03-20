//
// Created by 20159 on 2026/3/13.
//

#ifndef MOTOR_DM_H
#define MOTOR_DM_H


#include "struct_typedef.h"
#include "bsp_fdcan.h"

//电机返回时的CAN_id
#define motor_DM_master_id 0

//电机状态
enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
};

//控制模式(角度还是弧度)
enum Enum_Motor_DM_Mode
{
    Motor_DM_Pos_control = 0,
    Motor_DM_Angle_control,
};

//电机型号
enum motor_DM_Model{
	MOTOR_DM_J10010L = 0,
	MOTOR_DM_J4310	=1,
};

//返回数据
struct Struct_recv_motor_DM
{
    uint8_t MError;
    uint8_t mode;
    fp32 Now_Pos;                //角度
	fp32 Now_Angle;
    fp32 Now_W;                    //角速度
    fp32 Now_T;                    //力矩
    fp32 motor_Temperature;  
    fp32 mos_Temperature;          //温度
};

//发送数据
struct Struct_send_motor_DM
{
	fp32 Angle;
	fp32 Pos;
	fp32 T;
	fp32 W;	
	fp32 Kp;
	fp32 Kd;
};

//电机的控制结构体里面包含了电机的发送和电机的返回
struct motor_DM_control
{
	Struct_send_motor_DM send;
	Struct_recv_motor_DM recv;
};

//创建达妙电机类
class Class_Motor_DM{
	//公共
	public:

	//初始化
	void Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,const motor_DM_Model &__model,const Enum_Motor_DM_Mode &__control_mode);

	//电机的使能
	void enable();

	//电机的失能
	void lose();
	
	//电机零点设置
	void zero();

	//电机的发送
	void can_send();

	//电机的接收处理
	void can_recv( uint8_t *data);

	//以下为数据保护部分代码 外部访问内部变量需要运用Get和Set函数
	inline Enum_Motor_DM_Status Get_Status();

	inline void Set_Status(const Enum_Motor_DM_Status &__Status);

	inline void Set_Pos(const fp32 &__Pos);

	inline fp32 Get_Pos()const;

	inline void Set_Angle(const fp32 &__angle);

	inline fp32 Get_Angle()const;

	inline void Set_W(const fp32 &__w);

	inline fp32 Get_W()const;

	inline void Set_T(const fp32 &__T);

	inline fp32 Get_T()const;

	inline void Set_Kd(const fp32 &__Kd);

	inline fp32 Get_Kd()const;

	inline void Set_Kp(const fp32 &__Kp);

	inline fp32 Get_Kp()const;
	
	inline fp32 Get_Now_Angle()const;

	inline fp32 Get_Now_Pos()const;

	inline fp32 Get_Now_W()const;
	
	inline fp32 Get_Now_T()const;
	
	inline fp32 Get_motor_Temperature()const;

    inline fp32 Get_mos_Temperature()const;

	inline uint8_t Get_MError()const;
	
	inline uint8_t Get_mode()const;
	
	//内部
	protected:
	//CAN的id
	uint16_t CAN_id;
	//型号
	motor_DM_Model model;
	//can几
    FDCAN_HandleTypeDef *FDcan;
    // 最大位置, 与上位机控制幅值PMAX保持一致
	float Pos_Max;
    // 最大速度, 与上位机控制幅值VMAX保持一致
    float W_Max;
    // 最大扭矩, 与上位机控制幅值TMAX保持一致
    float T_Max;

	//电机状态
	Enum_Motor_DM_Status Motor_DM_Status = Motor_DM_Status_DISABLE;
	
	//返回数据
	Struct_recv_motor_DM recv;
	
	//控制模式
	Enum_Motor_DM_Mode control_mode = Motor_DM_Pos_control;

	fp32 Angle;
	fp32 Pos;
	fp32 T;
	fp32 W;	
	fp32 Kp;
	fp32 Kd;

};


inline Enum_Motor_DM_Status Class_Motor_DM::Get_Status()
{
	return (Motor_DM_Status);
}

inline void Class_Motor_DM::Set_Status(const Enum_Motor_DM_Status &__Status){
	Motor_DM_Status = __Status;
}

inline void Class_Motor_DM::Set_Angle(const fp32 &__angle){
	Angle = __angle;
}

inline fp32 Class_Motor_DM::Get_Angle() const{
	return (Angle);
}

inline void Class_Motor_DM::Set_W(const fp32 &__w){
	W=__w;
}

inline fp32 Class_Motor_DM::Get_W() const{
	return (W);
}

inline void Class_Motor_DM::Set_T(const fp32 &__T){
	T=__T;
}

inline fp32 Class_Motor_DM::Get_T()const{
	return (T);
}

inline void Class_Motor_DM::Set_Kd(const fp32 &__Kd){
	Kd = __Kd;
}

inline fp32 Class_Motor_DM::Get_Kd()const{
	return Kd;
}

inline void Class_Motor_DM::Set_Kp(const fp32 &__Kp){
	Kp = __Kp;
}

inline fp32 Class_Motor_DM::Get_Kp() const{
	return (Kp);
}

inline fp32 Class_Motor_DM::Get_Now_Angle()const{
	return (recv.Now_Angle);
}	

inline fp32 Class_Motor_DM::Get_Now_Pos()const{
	return (recv.Now_Pos);
}	

inline fp32 Class_Motor_DM::Get_Now_W()const{
	return (recv.Now_W);
}

inline fp32 Class_Motor_DM::Get_Now_T()const{
	return (recv.Now_T);
}

inline fp32 Class_Motor_DM::Get_motor_Temperature()const{
    return (recv.motor_Temperature);
}

inline fp32 Class_Motor_DM::Get_mos_Temperature()const{
    return (recv.mos_Temperature);
}

inline uint8_t Class_Motor_DM::Get_MError()const{
	return (recv.MError);
}
	
inline uint8_t Class_Motor_DM::Get_mode()const{
	return (recv.mode);
}

inline void Class_Motor_DM::Set_Pos(const fp32 &__Pos){
	Pos = __Pos;
}

inline fp32 Class_Motor_DM::Get_Pos()const{
	return Pos;
}


#endif //DM02_MOTOR_DM_H
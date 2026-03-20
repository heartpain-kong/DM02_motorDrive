//
// Created by 20159 on 2026/3/13.
//

#ifndef MOTOR_YS_H
#define MOTOR_YS_H


#include "struct_typedef.h"
#include "bsp_RS485.h"

//电机状态
enum Enum_Motor_YS_Status
{
    Motor_YS_Status_DISABLE = 0,
    Motor_YS_Status_ENABLE,
};

//控制模式(角度还是弧度)
enum Enum_Motor_YS_Mode
{
    Motor_YS_Pos_control = 0,
    Motor_YS_Angle_control,
};

//返回数据
struct Struct_recv_motor_YS
{
    uint8_t MError;
    uint8_t mode;
	fp32 Now_Pos;
    fp32 Now_Angle;                //角度
    fp32 Now_W;                    //角速度
    fp32 Now_T;                    //力矩
    fp32 Now_Temperature;          //温度
};

//发送数据
struct Struct_send_motor_YS
{
	fp32 Angle;
	fp32 Pos;
	fp32 T;
	fp32 W;	
	fp32 Kp;
	fp32 Kd;
};

//电机的控制结构体里面包含了电机的发送和电机的返回
struct motor_YS_control
{
	Struct_send_motor_YS send;
	Struct_recv_motor_YS recv;
};

//创建宇树电机类
class Class_Motor_YS{

	//公共
	public:

	//初始化
	void Init(UART_HandleTypeDef *huart,uint16_t __id,const Enum_Motor_YS_Mode &__control_mode);

	//电机的使能
	void enable();

	//电机的失能
	void lose();
	
	//电机零点设置
	void zero();

	//电机的发送
	void UART_send();

	//电机的接收处理
	void UART_recv(uint8_t *data);

	//以下为数据保护部分代码 外部访问内部变量需要运用Get和Set函数
	inline Enum_Motor_YS_Status Get_Status();

	inline void Set_Status(const Enum_Motor_YS_Status &__Status);

	inline void Set_Angle(const fp32 &__angle);

	inline fp32 Get_Angle()const;

	inline void Set_Init_Pos(const fp32 &__init_Pos);

	inline fp32 Get_Init_Pos()const;

	inline void Set_W(const fp32 &__w);

	inline fp32 Get_W()const;

	inline void Set_Pos(const fp32 &__Pos);

	inline fp32 Get_Pos()const;

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
	
	inline fp32 Get_Now_Temperature()const;

	inline uint8_t Get_MError()const;
	
	inline uint8_t Get_mode()const;
	
	//内部
	protected:
	//电机id
	uint16_t id;
	//串口编号
	UART_HandleTypeDef *UART;

	//电机状态
	Enum_Motor_YS_Status Motor_YS_Status = Motor_YS_Status_DISABLE;
	//返回数据
	Struct_recv_motor_YS recv;
	//控制模式
	Enum_Motor_YS_Mode control_mode = Motor_YS_Pos_control;

	uint8_t mode;
	//减速比
	fp32 Gearbox_Rate;

	fp32 init_Pos;
	fp32 init_Angle;
	fp32 Pos;
	fp32 Angle;
	fp32 T;
	fp32 W;	
	fp32 Kp;
	fp32 Kd;
	
	//发送数据的处理
	void send_data();
};


inline Enum_Motor_YS_Status Class_Motor_YS::Get_Status()
{
	return (Motor_YS_Status);
}

inline void Class_Motor_YS::Set_Status(const Enum_Motor_YS_Status &__Status){
	Motor_YS_Status = __Status;
}

inline void Class_Motor_YS::Set_Angle(const fp32 &__angle){
	Angle = __angle;
}

inline fp32 Class_Motor_YS::Get_Angle() const{
	return (Angle);
}

inline void Class_Motor_YS::Set_W(const fp32 &__w){
	W=__w;
}

inline fp32 Class_Motor_YS::Get_W() const{
	return (W);
}

inline void Class_Motor_YS::Set_T(const fp32 &__T){
	T=__T;
}

inline fp32 Class_Motor_YS::Get_T()const{
	return (T);
}

inline void Class_Motor_YS::Set_Kd(const fp32 &__Kd){
	Kd = __Kd;
}

inline fp32 Class_Motor_YS::Get_Kd()const{
	return Kd;
}

inline void Class_Motor_YS::Set_Kp(const fp32 &__Kp){
	Kp = __Kp;
}

inline fp32 Class_Motor_YS::Get_Kp() const{
	return (Kp);
}

inline fp32 Class_Motor_YS::Get_Now_Angle()const{
	return (recv.Now_Angle);
}	

inline fp32 Class_Motor_YS::Get_Now_W()const{
	return (recv.Now_W);
}

inline fp32 Class_Motor_YS::Get_Now_T()const{
	return (recv.Now_T);
}

inline fp32 Class_Motor_YS::Get_Now_Pos()const{
	return (recv.Now_Pos);
}	


inline fp32 Class_Motor_YS::Get_Now_Temperature()const{
	return (recv.Now_Temperature);
}

inline uint8_t Class_Motor_YS::Get_MError()const{
	return (recv.MError);
}
	
inline uint8_t Class_Motor_YS::Get_mode()const{
	return (recv.mode);
}

inline void Class_Motor_YS::Set_Init_Pos(const fp32 &__init_Pos){
	init_Pos = __init_Pos;
}

inline fp32 Class_Motor_YS::Get_Init_Pos()const{
	return init_Pos;

}

inline void Class_Motor_YS::Set_Pos(const fp32 &__Pos){
	Pos = __Pos;
}

inline fp32 Class_Motor_YS::Get_Pos()const{
	return Pos;
}


#endif //DM02_MOTOR_YS_H
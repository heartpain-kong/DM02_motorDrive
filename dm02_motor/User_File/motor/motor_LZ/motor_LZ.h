//
// Created by 20159 on 2026/3/13.
//

#ifndef MOTOR_LZ_H
#define MOTOR_LZ_H

#include "struct_typedef.h"
#include "bsp_fdcan.h"

//主机侧CAN_ID
#define motor_LZ_user_id 0

//电机状态
enum Enum_Motor_LZ_Status
{
    Motor_LZ_Status_DISABLE = 0,
    Motor_LZ_Status_ENABLE,
};

//控制模式(角度还是弧度)
enum Enum_Motor_LZ_Mode
{
    Motor_LZ_Pos_control = 0,
    Motor_LZ_Angle_control,
};

//电机型号
enum motor_LZ_Model{
	MOTOR_LZ_00=0,
	MOTOR_LZ_01,
	MOTOR_LZ_02,
	MOTOR_LZ_03,
	MOTOR_LZ_04,
	MOTOR_LZ_05,
};

//电机模式选择
enum canComMode{
	CANCOM_ANNOUNCE_DEVID = 0,//通告设备 ID
	CANCOM_MOTOR_CTRL, //MOTOR-电机控制
	CANCOM_MOTOR_FEEDBACK, //MOTOR-电机反馈
	CANCOM_MOTOR_IN, //MOTOR-进入电机模式
	CANCOM_MOTOR_RESET, //MOTOR-复位模式
	CANCOM_MOTOR_CALI, //MOTOR-高速编码器标定
	CANCOM_MOTOR_ZERO, //MOTOR-设置机械零位
	CANCOM_MOTOR_ID, //MOTOR-设置 ID
	CANCOM_PARA_WRITE, //参数-写入
	CANCOM_PARA_READ, //参数-读取
	CANCOM_CALI_ING, //编码器标定中
	CANCOM_CALI_RST, //编码器标定结果
	CANCOM_PARA_STR_INFO, //参数-字符串信息
	CANCOM_MOTOR_BRAKE, //MOTOR-进入刹车模式
	CANCOM_FAULT_WARN, //故障和警告信息
	CANCOM_MODE_TOTAL, //电机数据保存帧
	CANCOM_MODE_Bd,		//电机波特率修改帧
	CANCOM_MODE_ACTIVE_RECV = 0x18,//电机主动上报帧
	CANCOM_MODE_AGREEMENT,//电机协议修改帧
};

//返回数据
struct Struct_recv_motor_Lz
{
    uint8_t MError;
    uint8_t mode;
    fp32 Now_Pos;                //角度
	fp32 Now_Angle;
    fp32 Now_W;                    //角速度
    fp32 Now_T;                    //力矩
    fp32 Now_Temperature;          //温度
};

//发送数据
struct Struct_send_motor_Lz
{
	fp32 Angle;
	fp32 Pos;
	fp32 T;
	fp32 W;	
	fp32 Kp;
	fp32 Kd;
};

//电机的控制结构体里面包含了电机的发送和电机的返回
struct motor_lz_control
{
	Struct_send_motor_Lz send;
	Struct_recv_motor_Lz recv;
};

//创建灵足电机类
class Class_Motor_LZ{
	
	//公共
	public:

	//初始化
	void Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,const motor_LZ_Model &__model,const Enum_Motor_LZ_Mode &__control_mode);

	//电机的使能
	void enable();

	//电机的失能
	void lose();
	
	//电机主动上报1为开0为关
	void active_recv(uint8_t F_CMD);
	
	//电机零点设置
	void zero();

	//电机的发送
	void can_send();

	//电机的接收处理
	void can_recv(uint32_t Data , uint8_t *data);

	//电机的CAN_ID修改
	void motor_set_CAN_ID(uint8_t set_id);

	//以下为数据保护部分代码 外部访问内部变量需要运用Get和Set函数
	inline Enum_Motor_LZ_Status Get_Status();

	inline void Set_Status(const Enum_Motor_LZ_Status &__Status);
	
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
	
	inline fp32 Get_Now_Temperature()const;

	inline uint8_t Get_MError()const;
	
	inline uint8_t Get_mode()const;
	
	//内部
	protected:
	//CAN的id
	uint16_t CAN_id;
	//型号
	motor_LZ_Model model;
	//can几
	FDCAN_HandleTypeDef *FDcan;
    // 最大位置, 与上位机控制幅值PMAX保持一致
	float Pos_Max;
    // 最大速度, 与上位机控制幅值VMAX保持一致
    float W_Max;
    // 最大扭矩, 与上位机控制幅值TMAX保持一致
    float T_Max;

	//电机状态
	Enum_Motor_LZ_Status Motor_LZ_Status = Motor_LZ_Status_DISABLE;

	//返回数据
	Struct_recv_motor_Lz recv;
	
	//模式选择
	enum canComMode mode;
	
	//控制模式
	Enum_Motor_LZ_Mode control_mode = Motor_LZ_Pos_control;

	fp32 Angle;
	fp32 Pos;
	fp32 T;
	fp32 W;	
	fp32 Kp;

	fp32 Kd;
	//发送数据处理
	uint32_t send_data(uint32_t Id_data);
};


inline Enum_Motor_LZ_Status Class_Motor_LZ::Get_Status()
{
	return (Motor_LZ_Status);
}

inline void Class_Motor_LZ::Set_Status(const Enum_Motor_LZ_Status &__Status){
	Motor_LZ_Status = __Status;
}

inline void Class_Motor_LZ::Set_Angle(const fp32 &__angle){
	Angle = __angle;
}

inline fp32 Class_Motor_LZ::Get_Angle() const{
	return (Angle);
}

inline void Class_Motor_LZ::Set_W(const fp32 &__w){
	W=__w;
}

inline fp32 Class_Motor_LZ::Get_W() const{
	return (W);
}

inline void Class_Motor_LZ::Set_T(const fp32 &__T){
	T=__T;
}

inline fp32 Class_Motor_LZ::Get_T()const{
	return (T);
}

inline void Class_Motor_LZ::Set_Kd(const fp32 &__Kd){
	Kd = __Kd;
}

inline fp32 Class_Motor_LZ::Get_Kd()const{
	return Kd;
}

inline void Class_Motor_LZ::Set_Kp(const fp32 &__Kp){
	Kp = __Kp;
}

inline fp32 Class_Motor_LZ::Get_Kp() const{
	return (Kp);
}

inline fp32 Class_Motor_LZ::Get_Now_Angle()const{
	return (recv.Now_Angle);
}	

inline fp32 Class_Motor_LZ::Get_Now_Pos()const{
	return (recv.Now_Pos);
}	

inline fp32 Class_Motor_LZ::Get_Now_W()const{
	return (recv.Now_W);
}

inline fp32 Class_Motor_LZ::Get_Now_T()const{
	return (recv.Now_T);
}
	
inline fp32 Class_Motor_LZ::Get_Now_Temperature()const{
	return (recv.Now_Temperature);
}

inline uint8_t Class_Motor_LZ::Get_MError()const{
	return (recv.MError);
}
	
inline uint8_t Class_Motor_LZ::Get_mode()const{
	return (recv.mode);
}

inline void Class_Motor_LZ::Set_Pos(const fp32 &__Pos){
	Pos = __Pos;
}

inline fp32 Class_Motor_LZ::Get_Pos()const{
	return Pos;
}

#endif //MOTOR_LZ_H
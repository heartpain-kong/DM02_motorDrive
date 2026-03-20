#ifndef ALG_PID_H
#define ALG_PID_H

#include "struct_typedef.h"

enum Enum_PID_D_First
{
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
};

class Class_PID
{
public:
    void Init(const fp32 &__KP,const fp32 &__KI,const fp32 &__KD,const fp32 &__KF,const fp32 __max_out,const fp32 __max_iout);

    inline float Get_Out() const;

    inline void Set_KP(const fp32 &__KP);

    inline void Set_KI(const fp32 &__KI);

    inline void Set_KD(const fp32 &__KD);
   
    inline void Set_KF(const fp32 &__KF);

    inline void Set_Max_out(const fp32 &__max_out);

    inline void Set_Max_iout(const fp32 &__max_iout);

    inline void Set_Target(const fp32 &__Target);

    inline void Set_Now(const fp32 &__Now);

    inline void Set_Error();

    void Cout();

protected:

    // 目标值
    fp32 Target = 0.0f;
    // 当前值
    fp32 Now = 0.0f;
    // PID的P
    fp32 Kp=0.0f;
    // PID的I
    fp32 Ki=0.0f;
    // PID的D
    fp32 Kd=0.0f;
    // 前馈      3508 力位混             
    fp32 KF=0.0f;

    fp32 max_out; 
    fp32 max_iout; 

    fp32 out=0.0f;
    fp32 Pout=0.0f;
    fp32 Iout=0.0f;
    fp32 Dout=0.0f;
    fp32 Fout=0.0f;
    fp32 Dbuf[3];  
    fp32 error[3];

};

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
inline float Class_PID::Get_Out() const
{
    return (out);
}

/**
 * @brief 设定PID的P
 *
 * @param __K_P PID的P
 */
inline void Class_PID::Set_KP(const fp32 &__KP)
{
    Kp = __KP;
}

/**
 * @brief 设定PID的I
 *
 * @param __K_I PID的I
 */
inline void Class_PID::Set_KI(const fp32 &__KI)
{
    Ki = __KI;
}

/**
 * @brief 设定PID的D
 *
 * @param __K_D PID的D
 */
inline void Class_PID::Set_KD(const fp32 &__KD)
{
    Kd = __KD;
}

inline void Class_PID::Set_KF(const fp32 &__KF)
{
    KF = __KF;
}

/**
 * @brief 设定目标值
 *
 * @param __Target 目标值
 */
inline void Class_PID::Set_Target(const fp32 &__Target)
{
    Target = __Target;
}

/**
 * @brief 设定当前值
 *
 * @param __Now 当前值
 */
inline void Class_PID::Set_Now(const fp32 &__Now)
{
    Now = __Now;
}

/**
 * @brief 设定积分, 一般用于积分清零
 *
 * @param __Set_Integral_Error 积分值
 */
inline void Class_PID::Set_Error()
{
    error[0] = error[1] = error[2] = 0.0f;
    Dbuf[0] = Dbuf[1] = Dbuf[2] = 0.0f;
    out = Pout = Iout = Dout= Fout = 0.0f;
}

inline void Class_PID::Set_Max_iout(const fp32 &__max_iout)
{
    max_iout = __max_iout;
}

/**
 * @brief 设定输出限幅, 0为不限制
 *
 * @param __Out_Max 输出限幅, 0为不限制
 */
inline void Class_PID::Set_Max_out(const fp32 &__max_out)
{
    max_out = __max_out;
}


#endif
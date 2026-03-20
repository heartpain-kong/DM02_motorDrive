#include "alg_pid.h"
#include "math_support.h"

void Class_PID::Init(const fp32 &__KP,const fp32 &__KI,const fp32 &__KD,const fp32 &__KF,const fp32 __max_out,const fp32 __max_iout)
{
    Kp=__KP;
    Ki=__KI;
    Kd=__KD;
    KF=__KF;
    
    max_out=__max_out; 
    max_iout=__max_iout; 


    Dbuf[0] = Dbuf[1] = Dbuf[2] = 0.0f;
    error[0] = error[1] = error[2] = Pout = Iout = Dout = out = 0.0f;
}

void Class_PID::Cout()
{
    error[2] = error[1];
    error[1] = error[0];
    error[0] = Target - Now;
	Pout = Kp * error[0];
	Iout += Ki * error[0];
    Fout = KF * (error[0] - error[1] );
	Dbuf[2] = Dbuf[1];
	Dbuf[1] = Dbuf[0];
	Dbuf[0] = (error[0] - error[1]);
	Dout = Kd * Dbuf[0];
	LimitMax(Iout, max_iout);
	out = Pout + Iout + Dout + Fout;
	LimitMax(out, max_out);

}

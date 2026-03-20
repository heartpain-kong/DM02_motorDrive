//
// Created by 20159 on 2026/3/13.
//

#include "motor_DJ.h"
#include "math_support.h"

//dji的1拖4的地址数组
static uint8_t CAN_0x200_Tx_Data[8];
static uint8_t CAN_0x1ff_Tx_Data[8];
static uint8_t CAN_0x2ff_Tx_Data[8];

//确认那部分CAN有电机方便一起发送
bool CAN_Send_bl[3][3];

/**
 * @brief dji电机的初始化
 * @param hfdcan CAN编号
 * @param __id   CAN_ID
 * @param __model 电机的型号 MOTOR_DJ_M2006=0 & MOTOR_DJ_M3508 & MOTOR_DJ_M6020
 * @param __control_mode pid的控制模式  motor_DJ_Angle & motor_DJ_speed & motor_DJ_T
 * @return void
 */

void Class_Motor_DJ::Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,motor_DJ_Model __model,
        const motor_DJ_Control_mode &__control_mode)
{
    FDcan = hfdcan;
    CAN_id = __id;
    model = __model;
    Motor_DJ_Control_mode = __control_mode;
    if (hfdcan->Instance == FDCAN1)
    {
        CAN_Send_bl[0][CAN_id/4]=1;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        CAN_Send_bl[1][CAN_id/4]=1;
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        CAN_Send_bl[2][CAN_id/4]=1;
    }

    switch (model)
    {
        case MOTOR_DJ_M2006:
            Gearbox_Rate = 36.0f;
            // 扭矩电流常数
            CURRENT_TO_TORQUE = 0.18f / 36.0f;
            // 扭矩电流到输出的转化系数
            CURRENT_TO_OUT = 10000.0f / 10.0f;
            // 最大输出刻度
            OUT_MAX = 10000.0f;
        break;
        case MOTOR_DJ_M3508:

            Gearbox_Rate=19.0f;
            CURRENT_TO_TORQUE = 0.3f / (3591.0f / 187.0f);
            CURRENT_TO_OUT = 16384.0f / 20.0f;
            OUT_MAX = 16384.0f;  

        break;
        case MOTOR_DJ_M6020:

            Gearbox_Rate=1.0f;
            CURRENT_TO_TORQUE = 0.741f;
            CURRENT_TO_OUT = 16384.0f / 3.0f;
            OUT_MAX = 16384.0f;

        break;
    }
}

/**
 * @brief dji电机的数据的存储
 * @return void
 */
void Class_Motor_DJ::can_send_Data(){
    switch (CAN_id/4)
    {
        case 0:
            CAN_0x200_Tx_Data[CAN_id+CAN_id-2]=(int16_t)out>>8;
            CAN_0x200_Tx_Data[CAN_id+CAN_id-1]=out;
        break;
        case 1:
            CAN_0x1ff_Tx_Data[CAN_id+CAN_id-2-8]=(int16_t)out>>8;
            CAN_0x1ff_Tx_Data[CAN_id+CAN_id-1-8]=out;
        break;
        case 2:
            CAN_0x2ff_Tx_Data[CAN_id+CAN_id-2-16]=(int16_t)out>>8;
            CAN_0x2ff_Tx_Data[CAN_id+CAN_id-1-16]=out;
        break;
    
    }


}

/**
 * @brief dji电机的发送
 * @return void
 */
void motor_DJ_can_send(){
    if(CAN_Send_bl[0][0])fdcan_send_data_stand (&hfdcan1,CAN_M1_M4_ID,CAN_0x200_Tx_Data,8);
	if(CAN_Send_bl[0][1])fdcan_send_data_stand (&hfdcan1,CAN_M5_M8_ID,CAN_0x1ff_Tx_Data,8);
	if(CAN_Send_bl[0][2])fdcan_send_data_stand (&hfdcan1,CAN_M9_MA_ID,CAN_0x2ff_Tx_Data,8);
    if(CAN_Send_bl[1][0])fdcan_send_data_stand (&hfdcan2,CAN_M1_M4_ID,CAN_0x200_Tx_Data,8);
	if(CAN_Send_bl[1][1])fdcan_send_data_stand (&hfdcan2,CAN_M5_M8_ID,CAN_0x1ff_Tx_Data,8);
	if(CAN_Send_bl[1][2])fdcan_send_data_stand (&hfdcan2,CAN_M9_MA_ID,CAN_0x2ff_Tx_Data,8);
    if(CAN_Send_bl[2][0])fdcan_send_data_stand (&hfdcan3,CAN_M1_M4_ID,CAN_0x200_Tx_Data,8);
	if(CAN_Send_bl[2][1])fdcan_send_data_stand (&hfdcan3,CAN_M5_M8_ID,CAN_0x1ff_Tx_Data,8);
	if(CAN_Send_bl[2][2])fdcan_send_data_stand (&hfdcan3,CAN_M9_MA_ID,CAN_0x2ff_Tx_Data,8);
}

/**
 * @brief 运用pid处理数据
 * @return void
 */
void Class_Motor_DJ::PID_Calculate_Data(){
    switch (Motor_DJ_Control_mode)
    {
        case (motor_DJ_T):
        {
        break;
        }
        case (motor_DJ_speed):
        {
            PID_speed.Set_Target(W);
            PID_speed.Set_Now(recv.Now_W);
            PID_speed.Cout();

            T = PID_speed.Get_Out();

            break;
        }
        case (motor_DJ_Angle):
        {
            PID_Angle.Set_Target(Pos);
            PID_Angle.Set_Now(recv.Now_pos);
            PID_Angle.Cout();

            W = PID_Angle.Get_Out();

            PID_speed.Set_Target(W);
            PID_speed.Set_Now(recv.Now_W);
            PID_speed.Cout();

            T = PID_speed.Get_Out();

            break;
        }
        default:
        {
            T = 0.0f;

            break;
        }
    }
    out = (T) / CURRENT_TO_TORQUE / Gearbox_Rate * CURRENT_TO_OUT;
    motor_max_min(out, -OUT_MAX, OUT_MAX);
	can_send_Data();
}

/**
 * @brief dji电机的返回数据处理
 * @param *data 接收数组的指针 
 * @return void
 */    
void Class_Motor_DJ::can_recv(uint8_t *data){
    
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;

    tmp_encoder = (data[0]<<8|data[1]);
	tmp_omega =  (data[2]<<8|data[3]);
	tmp_current = (data[4]<<8|data[5]);

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - recv.Pre_Encoder;
    if (delta_encoder < -ENCODER_NUM_PER_ROUND / 2)
    {
        // 正方向转过了一圈
        recv.Total_Round++;
    }
    else if (delta_encoder > ENCODER_NUM_PER_ROUND / 2)
    {
        // 反方向转过了一圈
        recv.Total_Round--;
    }
    recv.Total_Encoder = recv.Total_Round * ENCODER_NUM_PER_ROUND + tmp_encoder + Encoder_Offset;

    // 计算电机本身信息
    recv.Now_pos = (float) recv.Total_Encoder / (float) ENCODER_NUM_PER_ROUND * 2.0f * PI / Gearbox_Rate;
    recv.Now_W = (float) tmp_omega * MATH_RPM_TO_RADPS /Gearbox_Rate;
    recv.Now_T = tmp_current / CURRENT_TO_OUT * CURRENT_TO_TORQUE *Gearbox_Rate;
    recv.Temperature = data[6];
   
    // 存储预备信息
    recv.Pre_Encoder = tmp_encoder;
    Motor_DJ_Status = Motor_Dj_Status_ENABLE;
}
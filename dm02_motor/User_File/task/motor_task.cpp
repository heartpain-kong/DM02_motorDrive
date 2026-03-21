#include "motor_task.h"
#include "motor_LZ.h"
#include "motor_DJ.h"
#include "motor_YS.h"
#include "motor_DM.h"
#include "crc_ccitt.h"
#include "tim.h"


extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

//灵足电机类和控制所需数据
Class_Motor_LZ motor_lz;
motor_lz_control motor_lz_data;

//大疆电机类和控制所需数据
Class_Motor_DJ motor_DJ;
motor_dj_control motor_dj_data;

//宇树电机类和控制所需数据
Class_Motor_YS motor_YS;
motor_YS_control motor_ys_data;

//达妙电机类和控制所需数据
Class_Motor_DM motor_dm;
motor_DM_control motor_dm_data;


void motor_LZ_Data_send(Class_Motor_LZ *__motor_lz,Struct_send_motor_Lz data);
void motor_DJ_Data_send(Class_Motor_DJ *__motor_dj,Struct_send_motor_DJ data);

void motor_YS_Data_send(Class_Motor_YS *__motor_YS,Struct_send_motor_YS data);
void motor_DM_Data_send(Class_Motor_DM *__motor_DM,Struct_send_motor_DM data);

void motor_LZ_Data_recv(Class_Motor_LZ *__motor_lz,Struct_recv_motor_Lz *data);
void motor_DJ_Data_recv(Class_Motor_DJ *__motor_dj,Struct_recv_motor_DJ *data);

void motor_YS_Data_recv(Class_Motor_YS *__motor_YS,Struct_recv_motor_YS *data);
void motor_DM_Data_recv(Class_Motor_DM *__motor_DM,Struct_recv_motor_DM *data);

/**
 * @brief CAN1的回调函数
 * @param Header can指针
 * @param *Buffer CAN接收数据数组
 * @return void
 */
void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
	
	
    uint8_t id = Header.Identifier >>8;
    uint8_t mode =(Header.Identifier>>24)&0x1F;
    switch (mode)
    {
		case 0x18:
		case 0x02:
        motor_lz.can_recv(Header.Identifier,Buffer);
        motor_LZ_Data_recv(&motor_lz,&motor_lz_data.recv);
		break;
		default:
		{
			break;
		}
    }
}

/**
 * @brief CAN2的回调函数
 * @param Header can指针
 * @param *Buffer CAN接收数据数组
 * @return void
 */
void CAN2_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch ( Header.Identifier)
	{
		
		case motor_DM_master_id:
		{			
			uint8_t DM_can_recv = Buffer[0] &0x0F;
			if(DM_can_recv==1){
				motor_dm.can_recv(Buffer);
				motor_DM_Data_recv(&motor_dm,&motor_dm_data.recv);
			}
		}

		default:
		{
			break;
		}
	}
}


/**
 * @brief CAN3的回调函数
 * @param Header can指针
 * @param *Buffer CAN接收数据数组
 * @return void
 */
void CAN3_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
	
    switch ( Header.Identifier)
	{
		
		case CAN_M1_ID:
		case CAN_M2_ID:
		case CAN_M3_ID:
		case CAN_M4_ID:
		case CAN_M5_ID:
		case CAN_M6_ID:
		case CAN_M7_ID:
		case CAN_M8_ID:
		case CAN_M9_ID:
		case CAN_MA_ID:
		case CAN_MB_ID:
		{
			motor_DJ.can_recv(Buffer); 
            motor_DJ_Data_recv(&motor_DJ,&motor_dj_data.recv);
			break;
		}

		default:
		{
			break;
		}
	}
}

/**
 * @brief 串口2的回调函数
 * @param Header 串口指针
 * @param *Buffer 串口收数据数组
 * @return void
 */
void USART2_RxHandler(UART_HandleTypeDef *Header, uint8_t *Buffer){
    uint16_t crc = crc_ccitt(Buffer,RS485_recv_Data_N-2);
    if((Buffer[14] != (crc&0xFF)) || (Buffer[15] != ((crc>>8) & 0xFF))){
        return;
    }
    uint8_t User_Rx_usart_id = Buffer[2]&0xF;
    switch (User_Rx_usart_id )
    {
    case  1:
        motor_YS.UART_recv(Buffer);
	    motor_YS_Data_recv(&motor_YS,&motor_ys_data.recv);
    break;
    default:
        break;
    }
}

/**
 * @brief 串口3的回调函数
 * @param Header 串口指针
 * @param *Buffer 串口收数据数组
 * @return void
 */
void USART3_RxHandler(UART_HandleTypeDef *Header, uint8_t *Buffer){
 
}

/**
 * @brief 1ms的回调函数
 * @return void
 */
void Task1ms_Callback(){
    
	motor_DM_Data_send (&motor_dm,motor_dm_data.send);
	motor_dm.can_send();
	
	motor_YS_Data_send(&motor_YS,motor_ys_data.send);
	motor_YS.UART_send();
	
}

/**
 * @brief 500us的回调函数
 * @return void
 */
void Task500us_Callback (){

}

/**
 * @brief 灵足电机的接收数据传递
 * @param __motor_lz 电机的类指针
 * @param *data 电机接收数据指针(外部定义方便计算)
 * @return void
 */
void motor_LZ_Data_recv(Class_Motor_LZ *__motor_lz,Struct_recv_motor_Lz *data){
    data->MError = __motor_lz->Get_MError();
    data->mode = __motor_lz->Get_mode();
    data->Now_Angle = __motor_lz->Get_Now_Angle();
    data->Now_Pos = __motor_lz->Get_Now_Pos();
    data->Now_W = __motor_lz->Get_Now_W();
    data->Now_T = __motor_lz->Get_Now_T();
    data->Now_Temperature = __motor_lz->Get_Now_Temperature();
}

/**
 * @brief 灵足电机的发送数据传递
 * @param __motor_lz 电机的类指针
 * @param data 电机发送数据(外部定义方便传递数据)
 * @return void
 */
void motor_LZ_Data_send(Class_Motor_LZ *__motor_lz,Struct_send_motor_Lz data){
    __motor_lz->Set_Angle(data.Angle);
    __motor_lz->Set_Pos(data.Pos);
    __motor_lz->Set_T(data.T);
    __motor_lz->Set_W(data.W);
    __motor_lz->Set_Kp(data.Kp);
    __motor_lz->Set_Kd(data.Kd);
}

/**
 * @brief 灵足电机的初始化
 * @param hfdcan can的指针
 * @param __motor_lz 电机的类指针
 * @param __model 电机型号
 * @param id 电机CAN_id
 * @param *data 电机接收数据指针(外部定义方便计算)
 * @return void
 */
void motor_LZ_Init(FDCAN_HandleTypeDef *hfdcan,Class_Motor_LZ *__motor_lz,const motor_LZ_Model __model,uint8_t id,Struct_recv_motor_Lz *data){
    __motor_lz->Init(hfdcan,id,__model,Motor_LZ_Pos_control);
    HAL_Delay(10);
    while(__motor_lz->Get_Now_Angle() ==0){
        __motor_lz->enable();
        
        HAL_Delay(1);
    }
    motor_LZ_Data_recv(__motor_lz,data);
    __motor_lz->active_recv(1);
    __motor_lz->Set_Status(Motor_LZ_Status_ENABLE);

}

/**
 * @brief 大疆电机的发送数据传递
 * @param __motor_dj 电机的类指针
 * @param data 电机发送数据(外部定义方便传递数据)
 * @return void
 */
void motor_DJ_Data_send(Class_Motor_DJ *__motor_dj,Struct_send_motor_DJ data){
    __motor_dj->Set_Pos(data.Pos);
    __motor_dj->Set_T(data.T);
    __motor_dj->Set_W(data.W);
}

/**
 * @brief 大疆电机的发送数据传递
 * @param __motor_dj 电机的类指针
 * @param *data 电机接收数据指针(外部定义方便计算)
 * @return void
 */
void motor_DJ_Data_recv(Class_Motor_DJ *__motor_dj,Struct_recv_motor_DJ *data){
    data->Now_pos = __motor_dj->Get_Now_pos();
    data->Now_T = __motor_dj->Get_Now_T();
    data->Now_W = __motor_dj->Get_Now_W();
}

/**
 * @brief 大疆电机的初始化
 * @param hfdcan can的指针
 * @param __motor_dj 电机的类指针
 * @param __model 电机型号
 * @param id 电机CAN_id
 * @param *data 电机接收数据指针(外部定义方便计算)
 * @return void
 */
void motor_DJ_Init(FDCAN_HandleTypeDef *hfdcan,Class_Motor_DJ *__motor_dj,const motor_DJ_Model __model,uint8_t id,Struct_recv_motor_DJ *data){
    __motor_dj->PID_speed.Init(0.015f,0.0005f,0.0f,0.0f,10000.0f,10000.0f);
    __motor_dj->PID_Angle.Init(1.0f,0.0f,0.02f,0.0f,10000.0f,10000.0f);
    __motor_dj->Init(hfdcan,id,__model,motor_DJ_speed);
    HAL_Delay(10);
    while(__motor_dj->Get_Now_pos() ==0){
        HAL_Delay(10);
    }
    motor_DJ_Data_recv(__motor_dj,data);
}

/**
 * @brief 宇数电机的初始化
 * @param huart uart的指针
 * @param __motor_YS 电机的类指针
 * @param id 电机id
 * @param *data 电机接收数据指针(外部定义方便计算)
 * @return void
 */
void motor_YS_Init(UART_HandleTypeDef *huart,Class_Motor_YS *__motor_YS,uint8_t id,Struct_recv_motor_YS *data){
     motor_YS.Init(huart,id,Motor_YS_Pos_control);
     motor_YS.enable();
     HAL_Delay(10);
     while(__motor_YS->Get_Angle() ==0){
         motor_YS.enable();
        HAL_Delay(1);
    }
    motor_YS_Data_recv(__motor_YS,data);
    motor_YS.zero();
    motor_YS.enable();
    HAL_Delay(1);
}

/**
 * @brief 宇数电机的接收数据传递
 * @param __motor_YS 电机的类指针
 * @param *data 电机接收数据指针(外部定义方便计算)
 * @return void
 */
void motor_YS_Data_recv(Class_Motor_YS *__motor_YS,Struct_recv_motor_YS *data){
    data->MError = __motor_YS->Get_MError();
    data->mode = __motor_YS->Get_mode();
    data->Now_Angle = __motor_YS->Get_Now_Angle();
    data->Now_Pos = __motor_YS->Get_Now_Pos();
    data->Now_W = __motor_YS->Get_Now_W();
    data->Now_T = __motor_YS->Get_Now_T();
    data->Now_Temperature = __motor_YS->Get_Now_Temperature();
}
/**
 * @brief 宇树电机的发送数据传递
 * @param __motor_YS 电机的类指针
 * @param data 电机发送数据(外部定义方便传递数据)
 * @return void
 */
void motor_YS_Data_send(Class_Motor_YS *__motor_YS,Struct_send_motor_YS data){
    __motor_YS->Set_Angle(data.Angle);
    __motor_YS->Set_Pos(data.Pos);
    __motor_YS->Set_T(data.T);
    __motor_YS->Set_W(data.W);
    __motor_YS->Set_Kp(data.Kp);
    __motor_YS->Set_Kd(data.Kd);
}

/**
 * @brief 达妙电机的接收数据传递
 * @param __motor_DM 电机的类指针
 * @param data 电机接收数据指针(外部定义方便计算)
 * @return void
 */
void motor_DM_Data_recv(Class_Motor_DM *__motor_DM,Struct_recv_motor_DM *data){
    data->MError = __motor_DM->Get_MError();
    data->mode = __motor_DM->Get_mode();
    data->Now_Angle = __motor_DM->Get_Now_Angle();
    data->Now_Pos = __motor_DM->Get_Now_Pos();
    data->Now_W = __motor_DM->Get_Now_W();
    data->Now_T = __motor_DM->Get_Now_T();
    data->motor_Temperature = __motor_DM->Get_motor_Temperature();
    data->mos_Temperature = __motor_DM->Get_mos_Temperature();

}


/**
 * @brief 达妙电机的发送数据传递
 * @param __motor_DM 电机的类指针
 * @param data 电机发送数据(外部定义方便传递数据)
 * @return void
 */
void motor_DM_Data_send(Class_Motor_DM *__motor_DM,Struct_send_motor_DM data){
    __motor_DM->Set_Angle(data.Angle);
    __motor_DM->Set_Pos(data.Pos);
    __motor_DM->Set_T(data.T);
    __motor_DM->Set_W(data.W);
    __motor_DM->Set_Kp(data.Kp);
    __motor_DM->Set_Kd(data.Kd);
}

/**
 * @brief 达妙电机的初始化
 * @param hfdcan can的指针
 * @param __motor_DM 电机的类指针
 * @param __model 电机型号
 * @param id 电机CAN_id
 * @param *data 电机接收数据指针(外部定义方便计算)
 * @return void
 */
void motor_DM_Init(FDCAN_HandleTypeDef *hfdcan,Class_Motor_DM *__motor_DM,const motor_DM_Model __model,uint8_t id,Struct_recv_motor_DM *data){
    __motor_DM->Init(hfdcan,id,__model,Motor_DM_Pos_control);
    HAL_Delay(10);
    
    while(__motor_DM->Get_Now_Pos() ==0){
        __motor_DM->enable();
        HAL_Delay(1);
    }
    motor_DM_Data_recv(__motor_DM,&motor_dm_data.recv);
    motor_dm_data.send.Pos = __motor_DM->Get_Now_Pos();
    motor_dm_data.send.Angle = __motor_DM->Get_Now_Angle();
    HAL_Delay(1);

}

/**
 * @brief 电机的初始化
 * @return void
 */
void motor_task_init(){

    //灵足电机的初始化
    bsp_can_init(&hfdcan1,CAN1_Callback);//CAN1回调函数的初始化
    motor_LZ_Init(&hfdcan1,&motor_lz,MOTOR_LZ_02,1,&motor_lz_data.recv);
    
    //大疆电机的初始化
    bsp_can_init(&hfdcan3,CAN3_Callback);//CAN2回调函数的初始化
    motor_DJ_Init(&hfdcan3,&motor_DJ,MOTOR_DJ_M3508,1,&motor_dj_data.recv);

    //达妙电机的初始化
    bsp_can_init(&hfdcan2,CAN2_Callback);//CAN3回调函数的初始化
    motor_DM_Init(&hfdcan2,&motor_dm,MOTOR_DM_J10010L,1,&motor_dm_data.recv);
    
    //宇树电机初始化
    USART_RX485_init(&huart2,USART2_RxHandler);//串口2回调函数的初始化
	motor_YS_Init(&huart2,&motor_YS,1,&motor_ys_data.recv);
	
    //开定时器7和8
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);
}

/**
 * @brief 电机任务
 * @return void
 */
void motor_task(){
    //灵足电机的发送
    motor_LZ_Data_send(&motor_lz,motor_lz_data.send);
    motor_lz.can_send();

    //大疆电机的数据传递
    motor_DJ_Data_send(&motor_DJ,motor_dj_data.send);
	motor_DJ.PID_Calculate_Data();
    motor_DJ_can_send ();
	
    HAL_Delay(1);
}


/**
 * @brief 定时器的中断回调函数
 * @param htim   定时器TIM_HandleTypeDef的指针
 * @return void
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        Task1ms_Callback();    //500ns的定时
    }
    else if (htim->Instance == TIM8)
    {
        Task500us_Callback();   //1ms的定时
    }
}





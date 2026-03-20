//
// Created by 20159 on 2026/3/12.
//

#include "bsp_fdcan.h"

//CAN_id初始化记录
bool system_can[3];

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

//CAN结构体
CAN_Manage_Object CAN1_Manage_Object;
CAN_Manage_Object CAN2_Manage_Object;
CAN_Manage_Object CAN3_Manage_Object;

/**
 * @brief CAN过滤器配置
 * @param hfdcan  can编号
 * @return void
 */
void can_filter_init(FDCAN_HandleTypeDef *hfdcan)
{
    
	FDCAN_FilterTypeDef fdcan_filter;
   
    fdcan_filter.IdType = FDCAN_STANDARD_ID;


    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(hfdcan,&fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(hfdcan,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, 1);
}

/**
 * @brief CAN的初始化
 * @param hfdcan  can编号
 * @param Callback_Function 回调函数传参
 * @return void
 */
void bsp_can_init(FDCAN_HandleTypeDef *hfdcan,CAN_Callback Callback_Function)
{
	if (hfdcan->Instance == FDCAN1)
		CAN1_Manage_Object.Callback_Function = Callback_Function;
    else if (hfdcan->Instance == FDCAN2)
        CAN2_Manage_Object.Callback_Function = Callback_Function;
    else if (hfdcan->Instance == FDCAN3)
        CAN3_Manage_Object.Callback_Function = Callback_Function;
    
	if(system_can[0]==0&&system_can[1]==0&&system_can[2]==0)can_filter_init(hfdcan);
    if (hfdcan->Instance == FDCAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hfdcan;
        if(system_can[0]==0){
            HAL_FDCAN_Start(hfdcan);    
			HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
			system_can[0]=1;
        }

    }
    else if (hfdcan->Instance == FDCAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hfdcan;
        if(system_can[1]==0){
			HAL_FDCAN_Start(hfdcan); 
			HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
            system_can[1]=1;
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        CAN3_Manage_Object.CAN_Handler = hfdcan;
        if(system_can[2]==0){
			HAL_FDCAN_Start(hfdcan); 
			HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
            system_can[2]=1;
        }
    }
}

/**
 * @brief CAN的标准帧发送
 * @param hfdcan  can编号
 * @param id CAN_id(标准帧)
 * @param *data 发送数据数组指针
 * @param len 数组长度
 * @return void
 */
uint8_t fdcan_send_data_stand(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len)
{
	FDCAN_TxHeaderTypeDef fdcan_tx_header;
    fdcan_tx_header.Identifier=id;
	fdcan_tx_header.IdType = FDCAN_STANDARD_ID;

    fdcan_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    fdcan_tx_header.DataLength = len;
    fdcan_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    fdcan_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    fdcan_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    fdcan_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    fdcan_tx_header.MessageMarker = 0;
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &fdcan_tx_header, data)!=HAL_OK)
        return 1;
    return 0;
}

/**
 * @brief CAN的扩展帧发送
 * @param hfdcan  can编号
 * @param id CAN_id(扩展帧)
 * @param *data 发送数据数组指针
 * @param len 数组长度
 * @return void
 */
uint8_t fdcan_send_data_Exten(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len)
{
	FDCAN_TxHeaderTypeDef fdcan_tx_header;
    fdcan_tx_header.Identifier=id;
	fdcan_tx_header.IdType = FDCAN_EXTENDED_ID;

    fdcan_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    fdcan_tx_header.DataLength = len;
    fdcan_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    fdcan_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    fdcan_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    fdcan_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    fdcan_tx_header.MessageMarker = 0;
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &fdcan_tx_header, data)!=HAL_OK)
        return 1;
	return 0;
}

/**
 * @brief CAN的接收回调函数
 * @param hfdcan  can编号
 * @param *data 接收数据数组指针
 * @return void
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

    if (hfdcan->Instance == FDCAN1)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN1_Manage_Object.Rx_Header, CAN1_Manage_Object.Rx_Buffer) == HAL_OK)
        {

            if (CAN1_Manage_Object.Callback_Function != nullptr)
            {
                CAN1_Manage_Object.Callback_Function(CAN1_Manage_Object.Rx_Header, CAN1_Manage_Object.Rx_Buffer);
            }
        }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN2_Manage_Object.Rx_Header, CAN2_Manage_Object.Rx_Buffer) == HAL_OK)
        {            

            if (CAN2_Manage_Object.Callback_Function != nullptr)
            {
                CAN2_Manage_Object.Callback_Function(CAN2_Manage_Object.Rx_Header, CAN2_Manage_Object.Rx_Buffer);
            }
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN3_Manage_Object.Rx_Header, CAN3_Manage_Object.Rx_Buffer) == HAL_OK)
        {
            

            if (CAN3_Manage_Object.Callback_Function != nullptr)
            {
                CAN3_Manage_Object.Callback_Function(CAN3_Manage_Object.Rx_Header, CAN3_Manage_Object.Rx_Buffer);
            }
        }
    }

}




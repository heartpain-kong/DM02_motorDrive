//
// Created by 20159 on 2026/3/12.
//

#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "fdcan.h"
#include "stm32h7xx_hal.h"

//定义CAN回调函数
typedef void (*CAN_Callback)(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

struct CAN_Manage_Object
{
    FDCAN_HandleTypeDef *CAN_Handler;
    CAN_Callback Callback_Function;
    // 与接收相关的数据
    FDCAN_RxHeaderTypeDef Rx_Header;
    uint8_t Rx_Buffer[8];
};

extern bool system_can[3];

void bsp_can_init(FDCAN_HandleTypeDef *hfdcan,CAN_Callback Callback_Function);

uint8_t fdcan_send_data_stand(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len);
uint8_t fdcan_send_data_Exten(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len);

#endif //BSP_FDCAN_H
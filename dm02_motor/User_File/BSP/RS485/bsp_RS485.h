//
// Created by 20159 on 2026/3/13.
//

#ifndef BSP_RS485_H
#define BSP_RS485_H

#include "usart.h"
#include "stm32h7xx_hal.h"

//宇树485发送数组大小
#define RS485_Send_Data_N 17
//宇树485接收数组大小
#define RS485_recv_Data_N 16


//定义串口回调函数
typedef void (*USART_Callback)(UART_HandleTypeDef *Header, uint8_t *Buffer);

void USART_RX485_init(UART_HandleTypeDef *huart,USART_Callback back);

uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data);


#endif
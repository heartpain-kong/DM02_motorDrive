//
// Created by 20159 on 2026/3/13.
//

#include "bsp_RS485.h"
#include <stm32h7xx_hal_def.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

USART_Callback Usart2_Callback_Function;
USART_Callback Usart3_Callback_Function;


//把数据放在0x24000000之后因为DMA原因
__attribute__((section (".AXI_SRAM"))) static uint8_t Usart2_recv_Data[RS485_recv_Data_N];
__attribute__((section (".AXI_SRAM"))) static uint8_t Usart3_recv_Data[RS485_recv_Data_N];

/**
 * @brief 串口的初始化
 * @param huart  串口编号
 * @param back 回调函数传参
 * @return void
 */
void USART_RX485_init(UART_HandleTypeDef *huart,USART_Callback back){
	HAL_RS485Ex_Init(huart,UART_DE_POLARITY_HIGH,2,2);
     if(huart==&huart2){
        Usart2_Callback_Function = back;
     }else if(huart==&huart3){
         Usart3_Callback_Function = back;
     }
    if(huart==&huart2){
        HAL_UART_Receive_DMA(huart,Usart2_recv_Data, RS485_recv_Data_N);
    }else if(huart==&huart3){
        HAL_UART_Receive_DMA(huart,Usart3_recv_Data, RS485_recv_Data_N);
    }

}

/**
 * @brief 串口的发送
 * @param huart  串口编号
 * @param *Data 发送数据数组指针
 * @return void
 */
uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data)
{
    __attribute__((section (".AXI_SRAM")))  static uint8_t Usart_send_Data[RS485_Send_Data_N];
    for(uint8_t i=0;i<RS485_Send_Data_N;++i)Usart_send_Data[i] = Data[i];

    return (HAL_UART_Transmit_DMA(huart, Usart_send_Data, RS485_Send_Data_N));
}

/**
 * @brief 串口接收回调函数
 * @param huart  串口编号
 * @return void
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart == &huart2)
    {

        if (Usart2_Callback_Function != nullptr)
        {
            Usart2_Callback_Function(huart, Usart2_recv_Data);
		
        }
        
    }
    else if (huart == &huart3)
    {
        if (Usart3_Callback_Function != nullptr)
        {
            Usart3_Callback_Function(huart, Usart3_recv_Data);
        }
    }
    
}

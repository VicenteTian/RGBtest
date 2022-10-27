#ifndef __UART_H
#define __UART_H

#include "stm32f1xx_hal.h"
#include "usart.h"
#include <stdio.h>
#define husart_debug     huart3        //printf所用串口号
void uart_putchar (UART_HandleTypeDef *huart,char ch);//发送一个字符
void uart_putstr (UART_HandleTypeDef *huart,char str[]);//发送字符串
void uart_putbuff (UART_HandleTypeDef *huart,uint8_t *buff, uint32_t len);//uart_putbuff (&huart1,(uint8_t *)"1234567", 7);
void vcan_sendware(void *wareaddr, uint32_t waresize);//发送数据波形至山外上位机虚拟示波器
void uart_getchar(UART_HandleTypeDef *huart,char *ch);
uint8_t uart_querychar (UART_HandleTypeDef *huart, char *ch);//查询是否接受到一个字节
uint8_t uart_querybuff (UART_HandleTypeDef *huart, char *buff, uint32_t max_len);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
#endif

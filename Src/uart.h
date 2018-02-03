#ifndef _UART_HEADER_
#define _UART_HEADER_

#include "stm32f4xx_hal.h"

#define USART_TX_AF GPIO_AF8_UART4
#define USART_RX_AF GPIO_AF8_UART4


void initUARTPins(GPIO_InitTypeDef* GPIO_InitStruct);
void initUART(UART_HandleTypeDef* UartHandle);
void initUARTInt(void);
void UARTINIT(GPIO_InitTypeDef* GPIOstruct, UART_HandleTypeDef* UARTstruct);

#endif

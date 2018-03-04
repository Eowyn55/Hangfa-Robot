#include "uart.h"


void initUARTPins(GPIO_InitTypeDef* GPIO_InitStruct){
	
	__GPIOA_CLK_ENABLE();
	//UART Transmit Pin Configuration
	GPIO_InitStruct->Pin       = GPIO_PIN_0;
  GPIO_InitStruct->Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct->Pull      = GPIO_NOPULL;
  GPIO_InitStruct->Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct->Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, GPIO_InitStruct);

  //UART Receive Pin Configuration
  GPIO_InitStruct->Pin = GPIO_PIN_1;
	GPIO_InitStruct->Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct->Pull      = GPIO_NOPULL;
  GPIO_InitStruct->Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct->Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, GPIO_InitStruct);
}

void initUART(UART_HandleTypeDef* huart4){
	__UART4_CLK_ENABLE();
	//Setting up and Enabling the UART4 Peripheral to be compatible with Bluetooth Click
  huart4->Instance = UART4;
  huart4->Init.BaudRate = 115200;
  huart4->Init.WordLength = UART_WORDLENGTH_8B;
  huart4->Init.StopBits = UART_STOPBITS_1;
  huart4->Init.Parity = UART_PARITY_NONE;
  huart4->Init.Mode = UART_MODE_TX_RX;
  huart4->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4->Init.OverSampling = UART_OVERSAMPLING_8;

  if (HAL_UART_Init(huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
//Enabling the interrupt
void initUARTInt(void){
	HAL_NVIC_SetPriority(UART4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(UART4_IRQn);

}

void UARTINIT(GPIO_InitTypeDef* GPIOstruct, UART_HandleTypeDef* UARTstruct){
	initUARTPins(GPIOstruct);
	initUART(UARTstruct);
	initUARTInt();
}


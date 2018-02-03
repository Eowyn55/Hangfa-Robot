#include "button.h"
GPIO_InitTypeDef GPIO_InitStruct2;


void initButton(void){
		__GPIOE_CLK_ENABLE();
	
	GPIO_InitStruct2.Pin       = GPIO_PIN_6;
  GPIO_InitStruct2.Mode      = GPIO_MODE_IT_RISING;
  GPIO_InitStruct2.Pull      = GPIO_NOPULL;
  GPIO_InitStruct2.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct2);
	
	
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}




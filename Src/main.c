/*
 *  Author       : Milica Stojiljkovic
 *  Project name : Autonomous vehicle
 *  Task name    : Main microcontroler 
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "button.h"
#include "uart.h"
#include "robotControl.h"

//Definition of variables used for CAN, UART, GPIO Structures
//CAN1 Base Address = 0x40006400
//UART4 Base Address = 0x40004C00
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef uartHandle;
TIM_HandleTypeDef htim10;
GPIO_InitTypeDef GPIO_InitStructx;
HAL_StatusTypeDef check = HAL_OK;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM10_Init(void);

// Time in ms to wait for another command
uint8_t time_ms = 0;
// cnt that counts every ms
uint16_t cnt_ms = 0;
// flag for stoping the movement (watchdog timer timeout)
uint8_t flag_stop = 0;

// Flag for uart interrupt 
uint8_t  flag_uart_main = 0;
uint8_t  flag_uart_timer = 0;

// flag for starting the countdown after the command is received
uint8_t flag_start_cnt = 0;

// counts 5 bytes for receiving commands
uint8_t  cnt_uart = 0;

//Init of buffers used for filling the CAN1 Mailbox
uint8_t abortBuffer[8];
uint8_t moveBuffer[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t moveBuffer_prev[4];

//Init of buffer used to recieve via the bluetooth module
uint8_t uartReceiveBfr[5];

//Status variable used to distinguish which operation to be sent to the robot
int status = 0;

//Macro to set the robot CAN 
#define set_extid_CAN(SubCommand, funcCode, addr)  ((uint32_t)((SubCommand<<16) | (funcCode<<8) | addr))

//button interrupt to start robot
void EXTI9_5_IRQHandler(void){
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_6);
	status = 1;
}

//Interrupt handler for the bluetooth click module
void UART4_IRQHandler(void){
	HAL_UART_IRQHandler(&uartHandle);
	cnt_uart = cnt_uart + 1;
	if (cnt_uart == 5)
	{
		cnt_uart = 0;
		flag_uart_main = 1;
		flag_uart_timer = 1;
		moveBuffer[0] = uartReceiveBfr[0];
		moveBuffer[1] = uartReceiveBfr[1];
		moveBuffer[2] = uartReceiveBfr[2];
		moveBuffer[3] = uartReceiveBfr[3];
		time_ms       = uartReceiveBfr[4];
  }
  HAL_UART_Receive_IT(&uartHandle, uartReceiveBfr, 5);		
}

/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim10); 
	if (flag_uart_timer == 1)
	{
		cnt_ms = 1;
		flag_uart_timer = 0;
		flag_start_cnt = 1;
	}
	else if (flag_start_cnt == 1)
	{
		cnt_ms = cnt_ms + 1;
		if (cnt_ms == time_ms*100 || cnt_ms == 9999)
		{
			flag_stop = 1;
			flag_start_cnt = 0;
		}
	}
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
	//Init CAN1 Peripheral and GPIO pins for CAN Rx and Tx
  MX_GPIO_Init();
  MX_CAN1_Init();
	MX_TIM10_Init();
	//Init GPIO push button used to turn 
	initButton();
	//Init of the UART pins and the UART peripheral, as well as enabling Interrupts
	UARTINIT(&GPIO_InitStructx, &uartHandle);
	//Filling all of the buffers
	int i;
	for(i = 0; i < 8; i++){
		abortBuffer[i] = 0x00;
	}
	//Creating variable of type transmit messgae from the CAN1 mailbox
	//Setting the pointer of the transmit mailbox to that address
	CanTxMsgTypeDef TxMessage;
	hcan1.pTxMsg = &TxMessage;
	//Filling the header fields of the CAN1 transmit Mailbox
	hcan1.pTxMsg->IDE = CAN_ID_EXT;
	hcan1.pTxMsg->StdId = 0;
	hcan1.pTxMsg->ExtId = set_extid_CAN(0x00,0x29,0x01); // 0x00 0x29 0x01 for manual speed control
	                                                     // 0x00 0x2A 0x01 for triaxial speed control
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 8;
	//Enable interrupt for the UART4 recieve
	HAL_UART_Receive_IT(&uartHandle, uartReceiveBfr, 5);
	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	HAL_TIM_Base_Start_IT(&htim10);
	abortMove(&hcan1);
	
	while(1)
	{	
		if (flag_uart_main == 1)
		{
			flag_uart_main = 0;
			
//			if ((moveBuffer[0] != moveBuffer_prev[0]) && (moveBuffer[1] != moveBuffer_prev[1]) &&
//				  (moveBuffer[2] != moveBuffer_prev[2]) && (moveBuffer[3] != moveBuffer_prev[3]))
//			{
				moveRobot(&hcan1, moveBuffer);
//				moveBuffer_prev[0] = moveBuffer[0];
//				moveBuffer_prev[1] = moveBuffer[1];
//				moveBuffer_prev[2] = moveBuffer[2];
//				moveBuffer_prev[3] = moveBuffer[3];
//			}
			
	  }
		if (flag_stop == 1)
		{
			flag_stop = 0;
			abortMove(&hcan1);
		}
	}
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_4TQ;
  hcan1.Init.BS2 = CAN_BS2_3TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{
	__TIM10_CLK_ENABLE();
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 16;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

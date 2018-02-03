/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "button.h"
#include "uart.h"
#include "robotControl.h"

//Definition of variables used for CAN, UART, GPIO Structures
//CAN1 Base Address = 0x40006400
//UART4 Base Address = 0x40004C00
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef uartHandle;
GPIO_InitTypeDef GPIO_InitStructx;
HAL_StatusTypeDef check = HAL_OK;

//TYPEDEVICE = 0 for Computer Bluetooth
//TYPEDEVICE = 1 for Android Bluetooth
#define TYPEDEVICE 0
//Definitions of the recieved hex numbers sent via bluetooth
#if TYPEDEVICE == 0
	#define FIGURE8 0x7F
	#define FORWARD 0x3F        
	#define GORIGHT 0x7E
	#define GOLEFT 0x1F                 
	#define STOP 0x7D
	#define BACKWARD 0x3E           
	#define CIRCLERIGHT 0x7C
	#define CIRCLELEFT 0x0F
	#define SLOW 0x7B
	#define MEDIUM 0x3D
	#define FAST 0x7A
#elif TYPEDEVICE == 1 
	#define FIGURE8 0x7F
	#define FORWARD 0xBF        
	#define GORIGHT 0x7E
	#define GOLEFT 0x5F                 
	#define STOP 0x7D
	#define BACKWARD 0xBE           
	#define CIRCLERIGHT 0x7C
	#define CIRCLELEFT 0xAF
	#define SLOW 0x7B
	#define MEDIUM 0xBD
	#define FAST 0x7A
#endif

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
//Init of buffers used for filling the CAN1 Mailbox
uint8_t abortBuffer[8];
uint8_t forwardBuffer[8];
uint8_t backwardBuffer[8];
uint8_t rotateBuffer[8];
uint8_t rotateBufferLeft[8];
uint8_t rotateBufferRight[8];
//Init of buffer used to recieve via the bluetooth module
uint8_t uartReceiveBfr[8];

//Status variable used to distinguish which operation to be sent to the robot
int status = 0;

//Macro to set the motherfucking shit nigga
#define set_extid_CAN(SubCommand, funcCode, addr)  ((uint32_t)((SubCommand<<16) | (funcCode<<8) | addr))

//button interrupt to start robot
void EXTI9_5_IRQHandler(void){
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_6);
	status = 1;
}

//Interrupt handler for the fucking bluetooth click module
void UART4_IRQHandler(void){
		HAL_UART_IRQHandler(&uartHandle);
		switch(uartReceiveBfr[0]){
			case FIGURE8:
				status = 1;
				break;
			case FORWARD:
				status = 2;
				break;
			case GORIGHT:
				status = 3;
				break;
			case GOLEFT:
				status = 4;
				break;
			case STOP:
				status = 5;
				abortMove(&hcan1, abortBuffer);
				break;
			case BACKWARD:
				status = 6;
				break;
			case CIRCLERIGHT:
				status = 7;
				break;
			case CIRCLELEFT:
				status = 8;
				break;
			case SLOW:
				status = 9;
				break;
			case MEDIUM:
				status = 10;
				break;
			case FAST:
				status = 11;
				break;
			default:
				status = -1;
		}
		HAL_UART_Receive_IT(&uartHandle, uartReceiveBfr, 1);		
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
	//Init CAN1 Peripheral and GPIO pins for CAN Rx and Tx
  MX_GPIO_Init();
  MX_CAN1_Init();
	//Init GPIO push button used to turn 
	initButton();
	//Init of the UART pins and the UART peripheral, as well as enabling Interrupts
	UARTINIT(&GPIO_InitStructx, &uartHandle);
	//Filling all of the buffers
	fillAbortBuffer(abortBuffer);
	fillForwardBuffer2(forwardBuffer);
	fillBackwardBuffer(backwardBuffer);
	fillrotateBufferLeft(rotateBufferLeft);
	fillrotateBufferRight(rotateBufferRight);
	//Creating variable of type transmit messgae from the CAN1 mailbox
	//Setting the pointer of the transmit mailbox to that address
	CanTxMsgTypeDef TxMessage;
	hcan1.pTxMsg = &TxMessage;
	//Filling the header fields of the CAN1 transmit Mailbox
	hcan1.pTxMsg->IDE = CAN_ID_EXT;
	hcan1.pTxMsg->StdId = 0;
	hcan1.pTxMsg->ExtId = set_extid_CAN(0x00,0x2A,0x01); // 0x00 0x29 0x01 for manual speed control
	                                                     // 0x00 0x2A 0x01 for triaxial speed control
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 8;
	//Enable interrupt for the UART4 recieve
	HAL_UART_Receive_IT(&uartHandle, uartReceiveBfr, 1);
	abortMove(&hcan1, abortBuffer);
	
	mediumSpeedX(forwardBuffer);
	goStraight(&hcan1, forwardBuffer);
		
	HAL_Delay(5000);
		
	mediumSpeedY(forwardBuffer);
	goStraight(&hcan1, forwardBuffer);
	
	HAL_Delay(5000);
	
	mediumSpeedX(forwardBuffer);
	goStraight(&hcan1, forwardBuffer);
	
	while(1)
	{	
		
		if(status == 1){
			status = -1;
			moveFigure8(&hcan1, forwardBuffer, backwardBuffer, rotateBuffer, abortBuffer);
		}
		else if(status == 2){
			status = -1;
			goStraight(&hcan1, forwardBuffer);
		}
		else if(status == 3){
			status = -1;
			goRight(&hcan1, rotateBufferRight);
		}
		else if(status == 4){
			status = -1;
			goLeft(&hcan1, rotateBufferLeft);
		}
		else if(status == 5){
			status = -1;
			abortMove(&hcan1, abortBuffer);
		}
		else if(status == 6){
			status = -1;
			goBack(&hcan1, backwardBuffer);
		}
		else if(status == 7){
			status = -1;
			circleRight(&hcan1);
		}
		else if(status == 8){
			status = -1;
			circleLeft(&hcan1);
		}
		else if(status == 9){
			status = -1;
			slowSpeedBackward(backwardBuffer);
			slowSpeedForward(forwardBuffer);
		}
		else if(status == 10){
			status = -1;
			mediumSpeedBackward(backwardBuffer);
			mediumSpeedForward(forwardBuffer);
		}
		else if(status == 11){
			status = -1;
			fastSpeedBackward(backwardBuffer);
			fastSpeedForward(forwardBuffer);
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

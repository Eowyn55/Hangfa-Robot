#include "robotControl.h"


void fillAbortBuffer(uint8_t *buffer){
		int j = 0;
		for(;j<8;j++)
			buffer[j] = 0x00;
}

void fillForwardBuffer2(uint8_t *buffer){
	buffer[0] = 0xE8;
	buffer[1] = 0x08;
	buffer[2] = 0x0F;
	buffer[3] = 0x0A;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}

void fillBackwardBuffer(uint8_t *buffer){
	buffer[0] = 0x0F;
	buffer[1] = 0x0A;
	buffer[2] = 0xE8;
	buffer[3] = 0x08;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}

void fillrotateBufferLeft(uint8_t *buffer){
	buffer[0] = 0xE8;
	buffer[1] = 0x04;
	buffer[2] = 0xE8;
	buffer[3] = 0x04;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}

void fillrotateBufferRight(uint8_t *buffer){
	buffer[0] = 0xEF;
	buffer[1] = 0x05;
	buffer[2] = 0xEF;
	buffer[3] = 0x05;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}

void slowSpeedForward(uint8_t *buffer){
	buffer[0] = 0xE8;
	buffer[1] = 0x03;
	buffer[2] = 0xFF;
	buffer[3] = 0x04;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}

void slowSpeedBackward(uint8_t *buffer){
	buffer[0] = 0xFF;
	buffer[1] = 0x04;
	buffer[2] = 0xE8;
	buffer[3] = 0x03;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}

void mediumSpeedForward(uint8_t *buffer){
//	buffer[0] = 0xE8;
//	buffer[1] = 0x08;
//	buffer[2] = 0x0F;
//	buffer[3] = 0x0A;
//	buffer[4] = 0x00;
//	buffer[5] = 0x00;
//	buffer[6] = 0x00;
//	buffer[7] = 0x00;
	buffer[0] = 0xE8;
	buffer[1] = 0x08;
	buffer[2] = 0xE8;
	buffer[3] = 0x08;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}

void mediumSpeedX(uint8_t *buffer){
	buffer[0] = 0xE8;
	buffer[1] = 0x08;
	buffer[2] = 0x00;
	buffer[3] = 0x00;
	buffer[4] = 0xFF;
	buffer[5] = 0x05;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}

void mediumSpeedY(uint8_t *buffer){
	buffer[0] = 0x00;
	buffer[1] = 0x00;
	buffer[2] = 0xE8;
	buffer[3] = 0x08;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}

void mediumSpeedBackward(uint8_t *buffer){
	buffer[0] = 0x0F;
	buffer[1] = 0x0A;
	buffer[2] = 0xE8;
	buffer[3] = 0x08;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}

void fastSpeedForward(uint8_t *buffer){
	buffer[0] = 0xE8;
	buffer[1] = 0x23;
	buffer[2] = 0xDF;
	buffer[3] = 0x23;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}

void fastSpeedBackward(uint8_t *buffer){
	buffer[0] = 0xDF;
	buffer[1] = 0x23;
	buffer[2] = 0xE8;
	buffer[3] = 0x23;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
}


HAL_StatusTypeDef abortMove(CAN_HandleTypeDef *hcan, uint8_t *buffer){
			HAL_StatusTypeDef tempCheck = HAL_OK;
			hcan->pTxMsg->Data[0] = buffer[0];
			hcan->pTxMsg->Data[1] = buffer[1];
			hcan->pTxMsg->Data[2] = buffer[2];
			hcan->pTxMsg->Data[3] = buffer[3];
			hcan->pTxMsg->Data[4] = buffer[4];
			hcan->pTxMsg->Data[5] = buffer[5];
			hcan->pTxMsg->Data[6] = buffer[6];
			hcan->pTxMsg->Data[7] = buffer[7];
			tempCheck = HAL_CAN_Transmit(hcan, 400);
			return tempCheck;
}

void rotateleft(CAN_HandleTypeDef *hcan, double angle,	uint8_t *abortBuffer){
	double circumference = 0.8953539;
	double distance = angle/360 * circumference;
	double duration = 1000*(distance/.228);

	hcan->pTxMsg->Data[0] = 0xE8;
	hcan->pTxMsg->Data[1] = 0x09;
	hcan->pTxMsg->Data[2] = 0xE8;
	hcan->pTxMsg->Data[3] = 0x09;
	hcan->pTxMsg->Data[4] = 0;
	hcan->pTxMsg->Data[5] = 0;
	hcan->pTxMsg->Data[6] = 0;
	hcan->pTxMsg->Data[7] = 0;
	if(HAL_CAN_Transmit(hcan, 400)!= HAL_OK){
		Error_Handler();
	}
	HAL_Delay(duration);
	abortMove(hcan, abortBuffer);

}

void rotateCCW(CAN_HandleTypeDef *hcan, int angle, float speed, uint8_t *buffer, uint8_t *abortBuffer){
		if(angle<0){
			speed=-speed;
		}
		float temp_length=0.2238;
		int duration= (int)(temp_length/speed*1000/90*angle);
		
		int16_t tmpSpeed = (int16_t)(speed* 10000);    
    buffer[0] = tmpSpeed & 0xFF;
    buffer[1] = tmpSpeed >> 8;
	
		hcan->pTxMsg->Data[0] = buffer[0];
		hcan->pTxMsg->Data[1] = buffer[1];
		hcan->pTxMsg->Data[2] = buffer[0];
		hcan->pTxMsg->Data[3] = buffer[1];
		hcan->pTxMsg->Data[4] = 0x00;// proveriti da li autorotational ima nekog smisla slati
		hcan->pTxMsg->Data[5] = 0x00;// 
		hcan->pTxMsg->Data[6] = 0x00;
		hcan->pTxMsg->Data[7] = 0x00;
		HAL_StatusTypeDef check = HAL_CAN_Transmit(hcan, 400);
		
		HAL_Delay(duration);
		
		abortMove(hcan, abortBuffer);
}

void rotateCW(CAN_HandleTypeDef *hcan, int angle, float speed, uint8_t *buffer, uint8_t *abortBuffer){
		// speed is in m/s and length is in m
	
		if(angle<0){
			speed=-speed;
		}
		
		float temp_length=0.2238;
		int duration= (int)(temp_length/speed*1000/90*angle);
		
		int16_t tmpSpeed = (int16_t)(speed* 10000);    
    buffer[0] = tmpSpeed & 0xFF;
    buffer[1] = tmpSpeed >> 8;
	
		hcan->pTxMsg->Data[0] = -buffer[0];
		hcan->pTxMsg->Data[1] = -buffer[1];
		hcan->pTxMsg->Data[2] = -buffer[0];
		hcan->pTxMsg->Data[3] = -buffer[1];
		hcan->pTxMsg->Data[4] = 0x00;// proveriti da li autorotational ima nekog smisla slati
		hcan->pTxMsg->Data[5] = 0x00;// 
		hcan->pTxMsg->Data[6] = 0x00;
		hcan->pTxMsg->Data[7] = 0x00;
		HAL_StatusTypeDef check = HAL_CAN_Transmit(hcan, 400);
		
		HAL_Delay(duration);
		
		abortMove(hcan, abortBuffer);
}
HAL_StatusTypeDef moveForward(CAN_HandleTypeDef *hcan, int duration, uint8_t *buffer){
		HAL_StatusTypeDef tempCheck = HAL_OK;
		hcan->pTxMsg->Data[0] = buffer[0];
		hcan->pTxMsg->Data[1] = buffer[1];
		hcan->pTxMsg->Data[2] = -buffer[2];
		hcan->pTxMsg->Data[3] = -buffer[3];
		hcan->pTxMsg->Data[4] = buffer[4];
		hcan->pTxMsg->Data[5] = buffer[5];
		hcan->pTxMsg->Data[6] = buffer[6];
		hcan->pTxMsg->Data[7] = buffer[7];
		tempCheck = HAL_CAN_Transmit(hcan, 400);
	
		HAL_Delay(duration);
	
		return tempCheck;
}

HAL_StatusTypeDef backwardMove(CAN_HandleTypeDef *hcan, int duration, uint8_t *buffer){
	
		HAL_StatusTypeDef tempCheck = HAL_OK;
		hcan->pTxMsg->Data[0] = -buffer[0];
		hcan->pTxMsg->Data[1] = -buffer[1];
		hcan->pTxMsg->Data[2] = buffer[2];
		hcan->pTxMsg->Data[3] = buffer[3];
		hcan->pTxMsg->Data[4] = buffer[4];
		hcan->pTxMsg->Data[5] = buffer[5];
		hcan->pTxMsg->Data[6] = buffer[6];
		hcan->pTxMsg->Data[7] = buffer[7];
		tempCheck = HAL_CAN_Transmit(hcan, 400);
	
		HAL_Delay(duration);
	
		return tempCheck;
}

void moveFigure8(CAN_HandleTypeDef *hcan, uint8_t *forwardbuffer, uint8_t *backwardbuffer, uint8_t *rotatebuffer, uint8_t *abortbuffer){
	
		if(moveForward(hcan, 6000, forwardbuffer) != HAL_OK){
			Error_Handler();
		}
		//abort();
		rotateCCW(hcan, 45, 0.1, rotatebuffer, abortbuffer); 
		if(backwardMove(hcan, 7500, backwardbuffer)!= HAL_OK){
			Error_Handler();
		}
			//abort();
		rotateCW(hcan, 75, 0.1, rotatebuffer, abortbuffer);
		if(moveForward(hcan, 6000, forwardbuffer) != HAL_OK){
			Error_Handler();
		}
			//abort();
		rotateCW(hcan, 65,0.1, rotatebuffer, abortbuffer);
		if(backwardMove(hcan, 7500, backwardbuffer)!= HAL_OK){
			Error_Handler();
		}
			//abort();
		rotateCCW(hcan, 55, 0.1, rotatebuffer, abortbuffer);	
}

void goStraight(CAN_HandleTypeDef *hcan, uint8_t *forwardBuffer){
//		hcan->pTxMsg->Data[0] = forwardBuffer[0];
//		hcan->pTxMsg->Data[1] = forwardBuffer[1];
//		hcan->pTxMsg->Data[2] = -forwardBuffer[2];
//		hcan->pTxMsg->Data[3] = -forwardBuffer[3];
//		hcan->pTxMsg->Data[4] = forwardBuffer[4];
//		hcan->pTxMsg->Data[5] = forwardBuffer[5];
//		hcan->pTxMsg->Data[6] = forwardBuffer[6];
//		hcan->pTxMsg->Data[7] = forwardBuffer[7];
	
	  hcan->pTxMsg->Data[0] = forwardBuffer[0];
		hcan->pTxMsg->Data[1] = forwardBuffer[1];
		hcan->pTxMsg->Data[2] = forwardBuffer[2];
		hcan->pTxMsg->Data[3] = forwardBuffer[3];
		hcan->pTxMsg->Data[4] = forwardBuffer[4];
		hcan->pTxMsg->Data[5] = forwardBuffer[5];
		hcan->pTxMsg->Data[6] = forwardBuffer[6];
		hcan->pTxMsg->Data[7] = forwardBuffer[7];
		HAL_CAN_Transmit(hcan, 400);
}

void goLeft(CAN_HandleTypeDef *hcan, uint8_t *rotateBuffer){
		hcan->pTxMsg->Data[0] = rotateBuffer[0];
		hcan->pTxMsg->Data[1] = rotateBuffer[1];
		hcan->pTxMsg->Data[2] = rotateBuffer[2];
		hcan->pTxMsg->Data[3] = rotateBuffer[3];
		hcan->pTxMsg->Data[4] = rotateBuffer[4];
		hcan->pTxMsg->Data[5] = rotateBuffer[5];
		hcan->pTxMsg->Data[6] = rotateBuffer[6];
		hcan->pTxMsg->Data[7] = rotateBuffer[7];
		HAL_CAN_Transmit(hcan, 400);
}

void goRight(CAN_HandleTypeDef *hcan, uint8_t *rotateBuffer){
		hcan->pTxMsg->Data[0] = -rotateBuffer[0];
		hcan->pTxMsg->Data[1] = -rotateBuffer[1];
		hcan->pTxMsg->Data[2] = -rotateBuffer[2];
		hcan->pTxMsg->Data[3] = -rotateBuffer[3];
		hcan->pTxMsg->Data[4] = rotateBuffer[4];
		hcan->pTxMsg->Data[5] = rotateBuffer[5];
		hcan->pTxMsg->Data[6] = rotateBuffer[6];
		hcan->pTxMsg->Data[7] = rotateBuffer[7];
		HAL_CAN_Transmit(hcan, 400);
}

void goBack(CAN_HandleTypeDef *hcan, uint8_t *backwardBuffer){
		hcan->pTxMsg->Data[0] = -backwardBuffer[0];
		hcan->pTxMsg->Data[1] = -backwardBuffer[1];
		hcan->pTxMsg->Data[2] = backwardBuffer[2];
		hcan->pTxMsg->Data[3] = backwardBuffer[3];
		hcan->pTxMsg->Data[4] = backwardBuffer[4];
		hcan->pTxMsg->Data[5] = backwardBuffer[5];
		hcan->pTxMsg->Data[6] = backwardBuffer[6];
		hcan->pTxMsg->Data[7] = backwardBuffer[7];
		HAL_CAN_Transmit(hcan, 400);
}

void circleLeft(CAN_HandleTypeDef *hcan){
		hcan->pTxMsg->Data[0] = 0xE8;
		hcan->pTxMsg->Data[1] = 0x28;
		hcan->pTxMsg->Data[2] = -0x0F;
		hcan->pTxMsg->Data[3] = -0x0A;
		hcan->pTxMsg->Data[4] = 0;
		hcan->pTxMsg->Data[5] = 0;
		hcan->pTxMsg->Data[6] = 0;
		hcan->pTxMsg->Data[7] = 0;
		HAL_CAN_Transmit(hcan, 400);
}

 void circleRight(CAN_HandleTypeDef *hcan){
		hcan->pTxMsg->Data[0] = 0xE8;
		hcan->pTxMsg->Data[1] = 0x08;
		hcan->pTxMsg->Data[2] = -0x0F;
		hcan->pTxMsg->Data[3] = -0x2A;
		hcan->pTxMsg->Data[4] = 0;
		hcan->pTxMsg->Data[5] = 0;
		hcan->pTxMsg->Data[6] = 0;
		hcan->pTxMsg->Data[7] = 0;
		HAL_CAN_Transmit(hcan, 400);
}
 
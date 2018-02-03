#ifndef __ROBOTCONTROL_H
#define __ROBOTCONTROL_H

#include "stm32f4xx_hal.h"

void fillAbortBuffer(uint8_t *buffer);
void fillForwardBuffer(uint8_t *buffer);
void fillBackwardBuffer(uint8_t *buffer);
void fillForwardBuffer2(uint8_t *buffer);
void fillrotateBufferRight(uint8_t *buffer);
void fillrotateBufferLeft(uint8_t *buffer);

void slowSpeedBackward(uint8_t *buffer);
void slowSpeedForward(uint8_t *buffer);
void mediumSpeedBackward(uint8_t *buffer);
void mediumSpeedForward(uint8_t *buffer);
void mediumSpeedX(uint8_t *buffer);
void mediumSpeedY(uint8_t *buffer);
void fastSpeedBackward(uint8_t *buffer);
void fastSpeedForward(uint8_t *buffer);

HAL_StatusTypeDef abortMove(CAN_HandleTypeDef *hcan, uint8_t * buffer);

void rotateleft(CAN_HandleTypeDef *hcan, double angle,	uint8_t *abortBuffer);

void rotateCW(CAN_HandleTypeDef *hcan, int angle, float speed, uint8_t * buffer, uint8_t *abortBuffer);

void rotateCCW(CAN_HandleTypeDef *hcan, int angle, float speed, uint8_t * buffer, uint8_t *abortBuffer);

HAL_StatusTypeDef moveForward(CAN_HandleTypeDef *hcan, int duration, uint8_t * buffer);

HAL_StatusTypeDef backwardMove(CAN_HandleTypeDef *hcan, int duration, uint8_t * buffer);

void moveFigure8(CAN_HandleTypeDef *hcan, uint8_t *forwardBuffer, uint8_t *backwardBuffer, uint8_t *rotateBuffer, uint8_t *abortbuffer);

void goStraight(CAN_HandleTypeDef *hcan, uint8_t *forwardBuffer);
//void goLeft(CAN_HandleTypeDef *hcan, uint8_t *forwardBuffer, uint8_t *rotateBuffer, uint8_t *abortbuffer);
//void goRight(CAN_HandleTypeDef *hcan, uint8_t *forwardBuffer, uint8_t *rotateBuffer, uint8_t *abortbuffer);
void goLeft(CAN_HandleTypeDef *hcan, uint8_t *rotateBuffer);
void goRight(CAN_HandleTypeDef *hcan, uint8_t *rotateBuffer);

void goBack(CAN_HandleTypeDef *hcan, uint8_t *backwardBuffer);

void circleLeft(CAN_HandleTypeDef *hcan);
void circleRight(CAN_HandleTypeDef *hcan);

#endif

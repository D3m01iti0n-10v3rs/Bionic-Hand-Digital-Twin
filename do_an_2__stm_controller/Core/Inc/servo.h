/*
 * servo.h
 *
 *  Created on: Sep 14, 2026
 *      Author: PC
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f1xx_hal.h"

typedef struct{
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	uint8_t angle;
	uint8_t target_angle;
} Servo_t;

Servo_t ServoInit(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t initial_angle);
void ServoWrite(Servo_t *Servo, uint8_t angle);
void ServoMoveStep(Servo_t *Servo, uint8_t target_angle, uint8_t angle_step);

uint8_t GetServoAngle(Servo_t *Servo);
uint8_t GetServoTargetAngle(Servo_t *Servo);

#endif /* INC_SERVO_H_ */

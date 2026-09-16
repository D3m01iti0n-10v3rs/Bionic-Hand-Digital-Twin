/*
 * servo.c
 *
 *  Created on: Sep 14, 2026
 *      Author: PC
 */

#include "servo.h"

Servo_t ServoInit(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t initial_angle){
	Servo_t Servo = {htim, channel, initial_angle};
    HAL_TIM_PWM_Start(htim, channel);
    ServoWrite(&Servo, initial_angle);
    return Servo;
}

void ServoWrite(Servo_t *Servo, uint8_t angle){
    if (angle > 180) angle = 180;
    Servo->angle = angle;

    uint32_t pulse = 1000 + (Servo->angle * 1000) / 180;
    __HAL_TIM_SET_COMPARE(Servo->htim, Servo->channel, pulse);
}

uint8_t GetServoAngle(Servo_t *Servo){
	return Servo->angle;
}

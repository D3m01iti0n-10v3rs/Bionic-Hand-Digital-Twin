/*
 * servo.c
 *
 *  Created on: Sep 14, 2026
 *      Author: PC
 */

#include "servo.h"

Servo_t ServoInit(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t initial_angle){
	Servo_t Servo = {htim, channel, initial_angle, initial_angle};
    HAL_TIM_PWM_Start(htim, channel);
    ServoWrite(&Servo, initial_angle);
    return Servo;
}

void ServoWrite(Servo_t *Servo, uint8_t angle){
    if (angle > 180) angle = 180;
    Servo->angle = angle;

    //uint32_t pulse = 1000 + (Servo->angle * 1000) / 180;
    uint32_t pulse = 500 + ((uint32_t)Servo->angle * 2000) / 180;
    __HAL_TIM_SET_COMPARE(Servo->htim, Servo->channel, pulse);
}

// make sure angle_step can have current angle into target angle
void ServoMoveStep(Servo_t *Servo, uint8_t target_angle, uint8_t angle_step){
    if (target_angle > 180) target_angle = 180;
    Servo->target_angle = target_angle;

    if (Servo->angle < Servo->target_angle){
        if (Servo->target_angle - Servo->angle <= angle_step) ServoWrite(Servo, Servo->target_angle);
        else ServoWrite(Servo, Servo->angle + angle_step);
    }

    else if (Servo->angle > Servo->target_angle){
        if (Servo->angle - Servo->target_angle <= angle_step) ServoWrite(Servo, Servo->target_angle);
        else ServoWrite(Servo, Servo->angle - angle_step);
    }
}

uint8_t GetServoAngle(Servo_t *Servo){
	return Servo->angle;
}

uint8_t GetServoTargetAngle(Servo_t *Servo){
	return Servo->target_angle;
}

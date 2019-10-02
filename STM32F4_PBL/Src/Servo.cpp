/*
 * Servo.cpp
 *
 *  Created on: Oct 1, 2019
 *      Author: piotrek
 */

#include "Servo.h"

#define LEFT(value) (IDLE_VALUE - value)
#define RIGHT(value) (IDLE_VALUE + value)

Servo::Servo()
{
}

Servo::~Servo()
{
	// TODO Auto-generated destructor stub
}

void Servo::setPWMOnTimer(uint16_t pwmValue)
{
	__HAL_TIM_SET_COMPARE(htim, timChannel, pwmValue);
}

void Servo::initialize(TIM_HandleTypeDef *htim, uint8_t timChannel)
{
	this->htim = htim;
	this->timChannel = timChannel;
}

void Servo::idle()
{
	setPWMOnTimer(IDLE_VALUE);
	this->pwmValue = IDLE_VALUE;
	servoStatus = idle_stat;
}

void Servo::left(uint16_t pwmValue)
{
	if(servoStatus == rightRun)
	{
		idle();
	}

	if (pwmValue > MAX_DIR_PWM)
	{
		pwmValue = MAX_DIR_PWM;
	}

	setPWMOnTimer(LEFT(pwmValue));
	this->pwmValue = pwmValue;
	servoStatus = leftRun;
}

void Servo::right(uint16_t pwmValue)
{
	if (servoStatus == leftRun)
	{
		idle();
	}

	if (pwmValue > MAX_DIR_PWM)
	{
		pwmValue = MAX_DIR_PWM;
	}

	setPWMOnTimer(RIGHT(pwmValue));
	this->pwmValue = pwmValue;
	servoStatus = rightRun;
}

void Servo::makeManeuver(uint8_t direction, uint16_t pwmData)
{
	switch (direction)
	{
	case 0:
		idle();
		break;

	case 1:
		left(pwmData);
		break;

	case 2:
		right(pwmData);
		break;

	default:
		break;
	}
}

void Servo::configureDesiredPWM(uint8_t *direction, uint16_t *desiredPWM)
{
	makeManeuver(*direction, *desiredPWM);
}

uint8_t Servo::getDataInArray(uint8_t *dataBuffer)
{
	uint8_t dataToReturn[SERVO_OBJECTDATAVOLUME];

	dataToReturn[0] = servoStatus;
	dataToReturn[1] = pwmValue >> 8;
	dataToReturn[2] = pwmValue & 0xFF;

	std::copy_n(dataToReturn, SERVO_OBJECTDATAVOLUME, dataBuffer);

	return SERVO_OBJECTDATAVOLUME;
}

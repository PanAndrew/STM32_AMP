/*
 * Servo.cpp
 *
 *  Created on: Oct 1, 2019
 *      Author: piotrek
 */

#include "Servo.h"

#ifdef UGV_4X4
	#define LEFT(value) (IDLE_VALUE + value)
	#define RIGHT(value) (IDLE_VALUE - value)
#else
	#define LEFT(value) (IDLE_VALUE - value)
	#define RIGHT(value) (IDLE_VALUE + value)
#endif

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
	this->pwmValue = 0;
	servoPresentStatus = idle_stat;
}

void Servo::left(uint16_t pwmValue)
{
	if(servoPresentStatus == rightRun)
	{
		idle();
	}

	if (pwmValue > MAX_DIR_PWM)
	{
		pwmValue = MAX_DIR_PWM;
	}

	setPWMOnTimer(LEFT(pwmValue));
	this->pwmValue = pwmValue;
	servoPresentStatus = leftRun;
}

void Servo::right(uint16_t pwmValue)
{
	if (servoPresentStatus == leftRun)
	{
		idle();
	}

	if (pwmValue > MAX_DIR_PWM)
	{
		pwmValue = MAX_DIR_PWM;
	}

	setPWMOnTimer(RIGHT(pwmValue));
	this->pwmValue = pwmValue;
	servoPresentStatus = rightRun;
}

void Servo::stop()
{
	configureDesiredPWM(&idleStateValue, &idlePWMValue);
}

void Servo::steeringIteration()
{
	makeManeuver(servoDesiredStatus, pwmDesiredValue);
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
	pwmDesiredValue = *desiredPWM;
	servoDesiredStatus = (status)*direction;
}

uint8_t Servo::getDataInArray(uint8_t *dataBuffer)
{
	uint8_t dataToReturn[SERVO_OBJECTDATAVOLUME];

	dataToReturn[0] = servoPresentStatus;
	dataToReturn[1] = pwmValue >> 8;
	dataToReturn[2] = pwmValue & 0xFF;

	std::copy_n(dataToReturn, SERVO_OBJECTDATAVOLUME, dataBuffer);

	return SERVO_OBJECTDATAVOLUME;
}

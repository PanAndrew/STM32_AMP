/*
 * DCMotorDriver.cpp
 *
 *  Created on: Oct 1, 2019
 *      Author: piotrek
 */

#include <DCMotorDriver.h>

#define FORWARD(value) (IDLE_VALUE + value)
#define BACK(value) (IDLE_VALUE - value)

DCMotorDriver::DCMotorDriver()
{
}

DCMotorDriver::~DCMotorDriver() {
	// TODO Auto-generated destructor stub
}

void DCMotorDriver::initialize(TIM_HandleTypeDef *htim, uint8_t timChannel)
{
	this->htim = htim;
	this->timChannel = timChannel;
}

void DCMotorDriver::setPWMOnTimer(uint16_t pwmValue)
{
	__HAL_TIM_SET_COMPARE(htim, timChannel, pwmValue);
}

void DCMotorDriver::idle()
{
	if(motorStatus == forwardRun)
	{
		braking();
	}

	setPWMOnTimer(IDLE_VALUE);
	this->pwmValue = IDLE_VALUE;
	motorStatus = idle_stat;
}

void DCMotorDriver::braking()
{
	setPWMOnTimer(BACK(BRAKE_PWM));
	this->pwmValue = BRAKE_PWM;
}

void DCMotorDriver::forward(uint16_t pwmValue)
{
	if(motorStatus == backRun)
	{
		idle();
	}

	if (pwmValue > MAX_DIR_PWM)
	{
		pwmValue = MAX_DIR_PWM;
	}

	setPWMOnTimer(FORWARD(pwmValue));
	this->pwmValue = pwmValue;
	motorStatus = forwardRun;
}

void DCMotorDriver::back(uint16_t pwmValue)
{
	if(motorStatus == forwardRun)
	{
		braking();
		idle();
	}

	if (pwmValue > MAX_DIR_PWM)
	{
		pwmValue = MAX_DIR_PWM;
	}

	setPWMOnTimer(BACK(pwmValue));
	this->pwmValue = pwmValue;
	motorStatus = backRun;
}

void DCMotorDriver::makeManeuver(uint8_t direction, uint16_t pwmData)
{
	switch (direction)
	{
	case 0:
		idle();
		break;

	case 1:
		forward(pwmData);
		break;

	case 2:
		back(pwmData);
		break;

	default:
		break;
	}
}

void DCMotorDriver::configureDesiredPWM(uint8_t *direction, uint16_t *desiredPWM)
{
	makeManeuver(*direction, *desiredPWM);
}

uint8_t DCMotorDriver::getDataInArray(uint8_t *dataBuffer)
{
	uint8_t dataToReturn[MOTOR_OBJECTDATAVOLUME];

	dataToReturn[0] = motorStatus;
	dataToReturn[1] = pwmValue >> 8;
	dataToReturn[2] = pwmValue & 0xFF;

	std::copy_n(dataToReturn, MOTOR_OBJECTDATAVOLUME, dataBuffer);

	return MOTOR_OBJECTDATAVOLUME;
}

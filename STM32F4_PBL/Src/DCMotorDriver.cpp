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
	setPWMOnTimer(IDLE_VALUE);
	this->pwmValue = 0;
	motorPresentStatus = idle_stat;
}

void DCMotorDriver::idling()
{
	setPWMOnTimer(IDLE_VALUE);
	this->pwmValue = 0;
	motorPresentStatus = idling_stat;
}

void DCMotorDriver::braking()
{
	if (motorPresentStatus == brakingFR_stat)
	{
		setPWMOnTimer(BACK(BRAKE_PWM));
		this->pwmValue = BRAKE_PWM;
	}
	else if (motorPresentStatus == brakingBR_stat)
	{
		setPWMOnTimer(IDLE_VALUE);
		this->pwmValue = 0;
	}
}

void DCMotorDriver::stop()
{
	configureDesiredPWM(&idleStateValue, &idlePWMValue);
}

bool DCMotorDriver::checkShiftingTime(uint16_t shiftingTime)
{
	if (HAL_GetTick() - shiftingTimestamp > shiftingTime) {
		return true;
	} else {
		return false;
	}
}

void DCMotorDriver::drivingIteration()
{
	if(!shiftingQueue.empty())
	{
		switch(shiftingQueue.front())
		{
		case brakingFR_stat:
			if(motorPresentStatus == brakingFR_stat)
			{
				if(checkShiftingTime(BREAKING_TIME_MS))
				{
					nextAction();
				} else {
					braking();
				}
			}
			else
			{
				motorPresentStatus = brakingFR_stat;
				braking();
				shiftingTimestamp = HAL_GetTick();
			}
			break;

		case brakingBR_stat:
			if(motorPresentStatus == brakingBR_stat)
			{
				if(checkShiftingTime(BREAKING_TIME_MS))
				{
					nextAction();
				} else {
					braking();
				}
			}
			else
			{
				motorPresentStatus = brakingBR_stat;
				braking();
				shiftingTimestamp = HAL_GetTick();
			}
			break;

		case idling_stat:
			if(motorPresentStatus == idling_stat)
			{
				if(checkShiftingTime(IDLING_TIME_MS))
				{
					nextAction();
				} else {
					idling();
				}
			}
			else
			{
				idling();
				shiftingTimestamp = HAL_GetTick();
			}
			break;
		default:
			makeManeuver(motorDesiredStatus, pwmDesiredValue);
			break;
		}
	}
	else
	{
		makeManeuver(motorDesiredStatus, pwmDesiredValue);
	}
}

bool DCMotorDriver::isShiftingQueueEmpty()
{
	return shiftingQueue.empty();
}

void DCMotorDriver::addToShiftingQueue(status statusToQueue)
{
	shiftingQueue.push_back(statusToQueue);
}

void DCMotorDriver::nextAction()
{
	shiftingQueue.pop_front();

	if(!shiftingQueue.empty())
	{
		idling();
		shiftingTimestamp = HAL_GetTick();
	}
	else
	{
		makeManeuver(motorDesiredStatus, pwmDesiredValue);
	}
}

void DCMotorDriver::forward(uint16_t pwmValue)
{
	if (pwmValue > MAX_DIR_PWM)
	{
		pwmValue = MAX_DIR_PWM;
	}

	setPWMOnTimer(FORWARD(pwmValue));
	this->pwmValue = pwmValue;
	motorPresentStatus = forwardRun;
}

void DCMotorDriver::back(uint16_t pwmValue)
{
	if (pwmValue > MAX_DIR_PWM)
	{
		pwmValue = MAX_DIR_PWM;
	}

	setPWMOnTimer(BACK(pwmValue));
	this->pwmValue = pwmValue;
	motorPresentStatus = backRun;
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
	pwmDesiredValue = *desiredPWM;
	motorDesiredStatus = (status)*direction;

	if(shiftingQueue.empty())
	{
		if(motorPresentStatus)
		{
			confShiftingQueue();
		}
	}
}

void DCMotorDriver::confShiftingQueue()
{
	if(motorDesiredStatus != motorPresentStatus)
	{
		if(motorDesiredStatus == idle_stat)
		{
			if(motorPresentStatus == forwardRun)
			{
				addToShiftingQueue(brakingFR_stat);
			}
			else
			{
				addToShiftingQueue(brakingBR_stat);
			}
		}
		else
		{
			if (motorPresentStatus == forwardRun)
			{
				addToShiftingQueue(brakingFR_stat);
				addToShiftingQueue(idling_stat);
			}
			else
			{
				addToShiftingQueue(brakingBR_stat);
			}
		}
	}
}

uint8_t DCMotorDriver::getDataInArray(uint8_t *dataBuffer)
{
	uint8_t dataToReturn[MOTOR_OBJECTDATAVOLUME];

	dataToReturn[0] = motorPresentStatus;
	dataToReturn[1] = pwmValue >> 8;
	dataToReturn[2] = pwmValue & 0xFF;

	std::copy_n(dataToReturn, MOTOR_OBJECTDATAVOLUME, dataBuffer);

	return MOTOR_OBJECTDATAVOLUME;
}

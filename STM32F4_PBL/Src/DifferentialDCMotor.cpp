/*
 * DifferentialDCMotor.cpp
 *
 *  Created on: Apr 25, 2020
 *      Author: piotrek
 */

#include "DifferentialDCMotor.h"

#define ADJUST(value) (value * 20)

DifferentialDCMotor::DifferentialDCMotor() {
	// TODO Auto-generated constructor stub

}

DifferentialDCMotor::~DifferentialDCMotor() {
	// TODO Auto-generated destructor stub
}

void DifferentialDCMotor::initialize(TIM_HandleTypeDef *htim,
		uint8_t leftTimChannel, uint8_t rightTimChannel)
{
	this->htim = htim;
	timChannel[LEFT_MOTOR] = leftTimChannel;
	timChannel[RIGHT_MOTOR] = rightTimChannel;
}

void DifferentialDCMotor::stop(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1,
		GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2)
{
	HAL_GPIO_WritePin(gpioBase_1, gpioPin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioBase_2, gpioPin_2, GPIO_PIN_RESET);
}

void DifferentialDCMotor::left(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1,
		GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2)
{
	HAL_GPIO_WritePin(gpioBase_1, gpioPin_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioBase_2, gpioPin_2, GPIO_PIN_RESET);
}

void DifferentialDCMotor::right(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1,
		GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2)
{
	HAL_GPIO_WritePin(gpioBase_1, gpioPin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioBase_2, gpioPin_2, GPIO_PIN_SET);
}

void DifferentialDCMotor::makeManeuver(uint8_t motorID, uint8_t maneuver, uint16_t pwmData)
{
	switch (maneuver)
	{
	case 0:
		if (motorID == LEFT_MOTOR)
		{
			stop_L();
		}
		else
		{
			stop_R();
		}
		break;

	case 1:
		if(motorID == LEFT_MOTOR)
			forward_L(pwmData);
		else
			forward_R(pwmData);
		break;

	case 2:
		if(motorID == LEFT_MOTOR)
			back_L(pwmData);
		else
			back_R(pwmData);
		break;

	default:
		break;
	}
}

void DifferentialDCMotor::setPWMOnTimer(uint8_t timChannel, uint16_t pwmValue)
{
	__HAL_TIM_SET_COMPARE(htim, timChannel, ADJUST(pwmValue));
}

void DifferentialDCMotor::forward_R(uint16_t pwmValue)
{
	if(motorPresentStatus[RIGHT_MOTOR] == backRun)
	{
		stop_R();
	}

	if(pwmValue > htim->Instance->ARR)
	{
		pwmValue = htim->Instance->ARR;
	}

	setPWMOnTimer(timChannel[RIGHT_MOTOR], ADJUST(pwmValue));
	this->pwmValue[LEFT_MOTOR] = pwmValue;
	right(BRIDGE_A1_GPIO_Port, BRIDGE_A1_Pin, BRIDGE_A2_GPIO_Port, BRIDGE_A2_Pin);
	motorPresentStatus[RIGHT_MOTOR] = forwardRun;
}

void DifferentialDCMotor::forward_L(uint16_t pwmValue)
{
	if(motorPresentStatus[LEFT_MOTOR] == backRun)
	{
		stop_L();
	}

	if(pwmValue > htim->Instance->ARR)
	{
		pwmValue = htim->Instance->ARR;
	}

	setPWMOnTimer(timChannel[LEFT_MOTOR], ADJUST(pwmValue));
	this->pwmValue[LEFT_MOTOR] = pwmValue;
	left(BRIDGE_B1_GPIO_Port, BRIDGE_B1_Pin, BRIDGE_B2_GPIO_Port, BRIDGE_B2_Pin);
	motorPresentStatus[LEFT_MOTOR] = forwardRun;
}

void DifferentialDCMotor::back_R(uint16_t pwmValue)
{
	if (motorPresentStatus[RIGHT_MOTOR] == forwardRun)
	{
		stop_R();
	}

	if(pwmValue > htim->Instance->ARR)
	{
		pwmValue = htim->Instance->ARR;
	}

	setPWMOnTimer(timChannel[RIGHT_MOTOR], ADJUST(pwmValue));
	this->pwmValue[RIGHT_MOTOR] = pwmValue;
	left(BRIDGE_A1_GPIO_Port, BRIDGE_A1_Pin, BRIDGE_A2_GPIO_Port, BRIDGE_A2_Pin);
	motorPresentStatus[RIGHT_MOTOR] = backRun;
}

void DifferentialDCMotor::back_L(uint16_t pwmValue)
{
	if(motorPresentStatus[LEFT_MOTOR] == forwardRun)
	{
		stop_L();
	}

	if(pwmValue > htim->Instance->ARR)
	{
		pwmValue = htim->Instance->ARR;
	}

	setPWMOnTimer(timChannel[LEFT_MOTOR], ADJUST(pwmValue));
	this->pwmValue[LEFT_MOTOR] = pwmValue;
	right(BRIDGE_B1_GPIO_Port, BRIDGE_B1_Pin, BRIDGE_B2_GPIO_Port, BRIDGE_B2_Pin);
	motorPresentStatus[LEFT_MOTOR] = backRun;
}

void DifferentialDCMotor::stop_R()
{
	stop(BRIDGE_A1_GPIO_Port, BRIDGE_A1_Pin, BRIDGE_A2_GPIO_Port, BRIDGE_A2_Pin);
	motorPresentStatus[RIGHT_MOTOR] = idle_stat;
	setPWMOnTimer(timChannel[RIGHT_MOTOR], idlePWMValue);
	this->pwmValue[RIGHT_MOTOR] = idlePWMValue;
}

void DifferentialDCMotor::stop_L()
{
	stop(BRIDGE_B1_GPIO_Port, BRIDGE_B1_Pin, BRIDGE_B2_GPIO_Port, BRIDGE_B2_Pin);
	motorPresentStatus[LEFT_MOTOR] = idle_stat;
	setPWMOnTimer(timChannel[LEFT_MOTOR], idlePWMValue);
	this->pwmValue[LEFT_MOTOR] = idlePWMValue;
}

void DifferentialDCMotor::drivingIteration()
{
	makeManeuver(LEFT_MOTOR, motorDesiredStatus[LEFT_MOTOR], pwmDesiredValue[LEFT_MOTOR]);
	makeManeuver(RIGHT_MOTOR, motorDesiredStatus[RIGHT_MOTOR], pwmDesiredValue[RIGHT_MOTOR]);
}

void DifferentialDCMotor::stopPlatform()
{
	stop_L();
	stop_R();
}

void DifferentialDCMotor::configureDesiredPWM(uint8_t *leftMotorDirection, uint8_t *rightMotorDirection, uint16_t *leftDesiredPWM, uint16_t *rightDesiredPWM)
{
	motorDesiredStatus[LEFT_MOTOR] = (status)*leftMotorDirection;
	motorDesiredStatus[RIGHT_MOTOR] = (status)*rightMotorDirection;

	pwmDesiredValue[LEFT_MOTOR] = *leftDesiredPWM;
	pwmDesiredValue[RIGHT_MOTOR] = *rightDesiredPWM;
}

uint8_t DifferentialDCMotor::getDataInArray(uint8_t *dataBuffer)
{
	uint8_t dataToReturn[DIFF_MOTOR_OBJECTDATAVOLUME];

	dataToReturn[0] = motorPresentStatus[LEFT_MOTOR] << 4 |  motorPresentStatus[RIGHT_MOTOR];
	dataToReturn[1] = pwmValue[LEFT_MOTOR] >> 8;
	dataToReturn[2] = pwmValue[LEFT_MOTOR] & 0xFF;
	dataToReturn[3] = pwmValue[RIGHT_MOTOR] >> 8;
	dataToReturn[4] = pwmValue[RIGHT_MOTOR] & 0xFF;

	std::copy_n(dataToReturn, DIFF_MOTOR_OBJECTDATAVOLUME, dataBuffer);

	return DIFF_MOTOR_OBJECTDATAVOLUME;
}

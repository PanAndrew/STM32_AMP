/*
 * DCMotor.cpp
 *
 *  Created on: May 15, 2019
 *      Author: piotrek
 */

#include "DCMotor.h"

#define ADJUST(value) (value * MAX_VOLTAGE / BATTERY_VOLTAGE)

DCMotor::DCMotor(TIM_HandleTypeDef *timer)
{
	this->htim = timer;
}

DCMotor::~DCMotor() {
	// TODO Auto-generated destructor stub
}

void DCMotor::stop(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1, GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2)
{
	HAL_GPIO_WritePin(gpioBase_1, gpioPin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioBase_2, gpioPin_2, GPIO_PIN_RESET);

}

void DCMotor::left(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1, GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2)
{
	HAL_GPIO_WritePin(gpioBase_1, gpioPin_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioBase_2, gpioPin_2, GPIO_PIN_RESET);
}

void DCMotor::right(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1, GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2)
{
	HAL_GPIO_WritePin(gpioBase_1, gpioPin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioBase_2, gpioPin_2, GPIO_PIN_SET);
}

inline void DCMotor::setPWMOnTimer(TIM_HandleTypeDef *htim, uint8_t timChannel, uint16_t pwmValue)
{
	__HAL_TIM_SET_COMPARE(htim, timChannel, ADJUST(pwmValue));
}

void DCMotor::forward_R(TIM_HandleTypeDef *htim, uint16_t pwmValue)
{
	if(motorStatus[RIGHT_MOTOR] == backRun)
	{
		stop_R();
	}

	if(pwmValue > htim->Instance->ARR + 1)
	{
		pwmValue = htim->Instance->ARR;
	}

	setPWMOnTimer(htim, TIM_CHANNEL_1, ADJUST(pwmValue));
	this->pwmValue[RIGHT_MOTOR] = pwmValue;
	right(BRIDGE_A1_GPIO_Port, BRIDGE_A1_Pin, BRIDGE_A2_GPIO_Port, BRIDGE_A2_Pin);
	motorStatus[RIGHT_MOTOR] = forwardRun;
}

void DCMotor::forward_L(TIM_HandleTypeDef *htim, uint16_t pwmValue)
{
	if(motorStatus[LEFT_MOTOR] == backRun)
	{
		stop_L();
	}

	if(pwmValue > htim->Instance->ARR + 1)
	{
		pwmValue = htim->Instance->ARR;
	}

	setPWMOnTimer(htim, TIM_CHANNEL_2, ADJUST(pwmValue));
	this->pwmValue[LEFT_MOTOR] = pwmValue;
	left(BRIDGE_B1_GPIO_Port, BRIDGE_B1_Pin, BRIDGE_B2_GPIO_Port, BRIDGE_B2_Pin);
	motorStatus[LEFT_MOTOR] = forwardRun;
}

void DCMotor::back_R(TIM_HandleTypeDef *htim, uint16_t pwmValue)
{
	if (motorStatus[RIGHT_MOTOR] == forwardRun)
	{
		stop_R();
	}

	if(pwmValue > htim->Instance->ARR + 1)
	{
		pwmValue = htim->Instance->ARR;
	}

	setPWMOnTimer(htim, TIM_CHANNEL_1, ADJUST(pwmValue));
	this->pwmValue[RIGHT_MOTOR] = pwmValue;
	left(BRIDGE_A1_GPIO_Port, BRIDGE_A1_Pin, BRIDGE_A2_GPIO_Port, BRIDGE_A2_Pin);
	motorStatus[RIGHT_MOTOR] = backRun;
}

void DCMotor::back_L(TIM_HandleTypeDef *htim, uint16_t pwmValue)
{
	if(motorStatus[LEFT_MOTOR] == forwardRun)
	{
		stop_L();
	}

	if(pwmValue > htim->Instance->ARR + 1)
	{
		pwmValue = htim->Instance->ARR;
	}

	setPWMOnTimer(htim, TIM_CHANNEL_2, ADJUST(pwmValue));
	this->pwmValue[LEFT_MOTOR] = pwmValue;
	right(BRIDGE_B1_GPIO_Port, BRIDGE_B1_Pin, BRIDGE_B2_GPIO_Port, BRIDGE_B2_Pin);
	motorStatus[LEFT_MOTOR] = backRun;
}

void DCMotor::stop_R(void)
{
	stop(BRIDGE_A1_GPIO_Port, BRIDGE_A1_Pin, BRIDGE_A2_GPIO_Port, BRIDGE_A2_Pin);
	motorStatus[RIGHT_MOTOR] = idle;
}

void DCMotor::stop_L(void)
{
	stop(BRIDGE_B1_GPIO_Port, BRIDGE_B1_Pin, BRIDGE_B2_GPIO_Port, BRIDGE_B2_Pin);
	motorStatus[LEFT_MOTOR] = idle;
}

void DCMotor::configureDesiredPWM(uint8_t* dataBuffer)
{
	uint8_t directions[2];
	directions[LEFT_MOTOR] = dataBuffer[DIRECTIONS_COMMAND] >> 4;
	directions[RIGHT_MOTOR] = dataBuffer[DIRECTIONS_COMMAND] & 0xF;

	uint16_t desiredPWMs[2];
	desiredPWMs[LEFT_MOTOR] = dataBuffer[LEFT_PWM_COMMAND_H] << 8 | dataBuffer[LEFT_PWM_COMMAND_L];
	desiredPWMs[RIGHT_MOTOR] = dataBuffer[RIGHT_PWM_COMMAND_H] << 8 | dataBuffer[RIGHT_PWM_COMMAND_L];

	makeManeuver(LEFT_MOTOR, directions[LEFT_MOTOR], desiredPWMs[LEFT_MOTOR]);
	makeManeuver(RIGHT_MOTOR, directions[RIGHT_MOTOR], desiredPWMs[RIGHT_MOTOR]);
}

void DCMotor::makeManeuver(uint8_t motorID, uint8_t maneuver, uint16_t pwmData)
{
	switch (maneuver)
	{
		case 0:
			if (motorID == LEFT_MOTOR)
			{
				stop_L();
				setPWMOnTimer(htim, TIM_CHANNEL_2, pwmData);
			}
			else
			{
				stop_R();
				setPWMOnTimer(htim, TIM_CHANNEL_1, pwmData);
			}
			break;

		case 1:
			if(motorID == LEFT_MOTOR)
				forward_L(htim, pwmData);
			else
				forward_R(htim, pwmData);
			break;

		case 2:
			if(motorID == LEFT_MOTOR)
				back_L(htim, pwmData);
			else
				back_R(htim, pwmData);
			break;

		default:
			break;
	}
}

uint8_t DCMotor::getDataInArray(uint8_t* dataBuffer)
{
	uint8_t dataToReturn[MOTOR_OBJECTDATAVOLUME];

	dataToReturn[0] = (motorStatus[LEFT_MOTOR] << 4) | motorStatus[RIGHT_MOTOR];
	dataToReturn[1] = pwmValue[LEFT_MOTOR] >> 8;
	dataToReturn[2] = pwmValue[LEFT_MOTOR] & 0xFF;
	dataToReturn[3] = pwmValue[RIGHT_MOTOR] >> 8;
	dataToReturn[4] = pwmValue[RIGHT_MOTOR] & 0xFF;

	std::copy_n(dataToReturn, MOTOR_OBJECTDATAVOLUME, dataBuffer);

	return MOTOR_OBJECTDATAVOLUME;
}

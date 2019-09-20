/*
 * DCMotor.cpp
 *
 *  Created on: May 15, 2019
 *      Author: piotrek
 */

#include "DCMotor.h"

DCMotor::DCMotor() {
	// TODO Auto-generated constructor stub

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

void DCMotor::forward_R(TIM_HandleTypeDef *htim, uint16_t pwmValue)
{
	if(motorStatus[RIGHT_MOTOR] == backRun)
	{
		stop_R();
	}
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pwmValue);
	right(BRIDGE_A1_GPIO_Port, BRIDGE_A1_Pin, BRIDGE_A2_GPIO_Port, BRIDGE_A2_Pin);
	motorStatus[RIGHT_MOTOR] = forwardRun;
}

void DCMotor::forward_L(TIM_HandleTypeDef *htim, uint16_t pwmValue)
{
	if(motorStatus[LEFT_MOTOR] == backRun)
	{
		stop_L();
	}
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pwmValue);
	left(BRIDGE_B1_GPIO_Port, BRIDGE_B1_Pin, BRIDGE_B2_GPIO_Port, BRIDGE_B2_Pin);
	motorStatus[LEFT_MOTOR] = forwardRun;
}

void DCMotor::back_R(TIM_HandleTypeDef *htim, uint16_t pwmValue)
{
	if (motorStatus[RIGHT_MOTOR] == forwardRun)
	{
		stop_R();
	}
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pwmValue);
	left(BRIDGE_A1_GPIO_Port, BRIDGE_A1_Pin, BRIDGE_A2_GPIO_Port, BRIDGE_A2_Pin);
	motorStatus[RIGHT_MOTOR] = backRun;
}

void DCMotor::back_L(TIM_HandleTypeDef *htim, uint16_t pwmValue)
{
	if(motorStatus[LEFT_MOTOR] == forwardRun)
	{
		stop_L();
	}
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pwmValue);
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

uint8_t DCMotor::getDataInArray(uint8_t* dataBuffer)
{
	uint8_t dataToReturn[MOTOR_OBJECTDATAVOLUME];

	dataToReturn[0] = (motorStatus[LEFT_MOTOR] << 4) | motorStatus[RIGHT_MOTOR];

	std::copy_n(pwmValue, sizeof pwmValue, dataToReturn);

	std::copy_n(dataToReturn, MOTOR_OBJECTDATAVOLUME, dataBuffer);

	return MOTOR_OBJECTDATAVOLUME;
}

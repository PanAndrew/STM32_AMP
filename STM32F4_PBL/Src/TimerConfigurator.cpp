/*
 * TimerConfigurator.cpp
 *
 *  Created on: Nov 13, 2019
 *      Author: piotrek
 */

#include "TimerConfigurator.h"

TimerConfigurator::TimerConfigurator(TIM_HandleTypeDef* htim_IMU,
									TIM_HandleTypeDef* htim_DataSend,
									TIM_HandleTypeDef* htim_Led)
{
	htimIMU = htim_IMU;
	htimDataSend = htim_DataSend;
	htimLed = htim_Led;
}

TimerConfigurator::~TimerConfigurator() {
	// TODO Auto-generated destructor stub
}

void TimerConfigurator::configureTimer(uint8_t* dataBuffer)
{
	uint8_t timerID;
	uint16_t prescalerValue;
	uint16_t ARRValue;
	uint8_t clockDividerValue;

	timerID = dataBuffer[TIMER_ID];
	prescalerValue = dataBuffer[PRESCALER_VALUE_H] << 8 | dataBuffer[PRESCALER_VALUE_L];
	ARRValue = dataBuffer[ARR_VALUE_H] << 8 | dataBuffer[ARR_VALUE_L];
	clockDividerValue = dataBuffer[CLOCK_DIVIDER];

	manageConfiguration(timerID, prescalerValue, ARRValue, clockDividerValue);
}

void TimerConfigurator::manageConfiguration(uint8_t &timerID, uint16_t &prescalerValue,
		uint16_t &ARRValue, uint8_t &clockDividerValue)
{
	switch(timerID)
	{
	case HTIM_IMU_ID:
		setConfigurationTim(htimIMU, prescalerValue, ARRValue);
		break;

	case HTIM_DATA_SEND_ID:
		setConfigurationAdvTim(htimDataSend, prescalerValue, ARRValue, clockDividerValue);
		break;

	case HTIM_LED:
		setConfigurationTim(htimLed, prescalerValue, ARRValue);
		break;

	default:
		break;
	}
}

void TimerConfigurator::setConfigurationAdvTim(TIM_HandleTypeDef* htim, uint16_t &prescalerValue,
			uint16_t &ARRValue, uint8_t &clockDividerValue)
{
	HAL_TIM_Base_Stop_IT(htim);
	__HAL_TIM_SET_PRESCALER(htim, prescalerValue);
	__HAL_TIM_SET_AUTORELOAD(htim, ARRValue);

	if(clockDividerValue == 1)
	{
		__HAL_TIM_SET_CLOCKDIVISION(htim, TIM_CLOCKDIVISION_DIV1);
	}
	else if(clockDividerValue == 2)
	{
		__HAL_TIM_SET_CLOCKDIVISION(htim, TIM_CLOCKDIVISION_DIV2);
	}
	else if(clockDividerValue == 4)
	{
		__HAL_TIM_SET_CLOCKDIVISION(htim, TIM_CLOCKDIVISION_DIV4);
	}
	else
	{
		__HAL_TIM_SET_CLOCKDIVISION(htim, TIM_CLOCKDIVISION_DIV1);
	}

	__HAL_TIM_SET_COUNTER(htim, 0);
	HAL_TIM_Base_Start_IT(htim);
}

void TimerConfigurator::setConfigurationTim(TIM_HandleTypeDef* htim, uint16_t &prescalerValue,
				uint16_t &ARRValue)
{
	HAL_TIM_Base_Stop_IT(htim);
	__HAL_TIM_SET_PRESCALER(htim, prescalerValue);
	__HAL_TIM_SET_AUTORELOAD(htim, ARRValue);
	__HAL_TIM_SET_COUNTER(htim, 0);
	HAL_TIM_Base_Start_IT(htim);
}

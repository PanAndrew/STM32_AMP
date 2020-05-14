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

float TimerConfigurator::calcFreq(uint32_t clock, uint16_t psc, uint16_t arr)
{
	return (float)clock/(((float)psc + 1.0)*((float)arr + 1.0));
}

uint8_t TimerConfigurator::getDataInArray(uint8_t *dataBuffer)
{
	uint8_t dataToReturn[TIM_CONF_OBJECTDATAVOLUME];

	dataToReturn[0] = htimIMU->Instance->PSC >> 8;
	dataToReturn[1] = htimIMU->Instance->PSC & 0xFF;
	dataToReturn[2] = htimIMU->Instance->ARR >> 8;
	dataToReturn[3] = htimIMU->Instance->ARR & 0xFF;
	dataToReturn[4] = htimIMU->Init.ClockDivision;
	floatValTemp.floatVal = calcFreq(90000000, htimIMU->Instance->PSC, htimIMU->Instance->ARR);
	dataToReturn[5] = floatValTemp.arrVal[0];
	dataToReturn[6] = floatValTemp.arrVal[1];
	dataToReturn[7] = floatValTemp.arrVal[2];
	dataToReturn[8] = floatValTemp.arrVal[3];
	dataToReturn[9] = htimDataSend->Instance->PSC >> 8;
	dataToReturn[10] = htimDataSend->Instance->PSC & 0xFF;
	dataToReturn[11] = htimDataSend->Instance->ARR >> 8;
	dataToReturn[12] = htimDataSend->Instance->ARR & 0xFF;
	dataToReturn[13] = htimDataSend->Init.ClockDivision;
	floatValTemp.floatVal = calcFreq(180000000, htimDataSend->Instance->PSC, htimDataSend->Instance->ARR);
	dataToReturn[14] = floatValTemp.arrVal[0];
	dataToReturn[15] = floatValTemp.arrVal[1];
	dataToReturn[16] = floatValTemp.arrVal[2];
	dataToReturn[17] = floatValTemp.arrVal[3];

	std::copy_n(dataToReturn, TIM_CONF_OBJECTDATAVOLUME, dataBuffer);

	return TIM_CONF_OBJECTDATAVOLUME;
}

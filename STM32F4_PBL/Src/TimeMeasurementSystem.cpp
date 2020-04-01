/*
 * TimeMeasurementSystem.cpp
 *
 *  Created on: Mar 31, 2020
 *      Author: piotrek
 */

#include "TimeMeasurementSystem.h"

TimeMeasurementSystem::TimeMeasurementSystem(TIM_HandleTypeDef *htim)
{
	timerHandle = htim;
}

TimeMeasurementSystem::~TimeMeasurementSystem() {
	// TODO Auto-generated destructor stub
}

void TimeMeasurementSystem::takeTS(uint8_t choosenCell)
{
	timestamps[choosenCell] = timerHandle->Instance->CNT;
}

void TimeMeasurementSystem::calculateElapsedTime(uint8_t choosenCell)
{
	uint16_t diff = timerHandle->Instance->CNT - timestamps[choosenCell];

	if(diff >= 0)
	{
		elapsedTime[choosenCell] += diff;
	}
	else
	{
		elapsedTime[choosenCell] += diff + timerHandle->Instance->ARR + 1;
	}
}

uint8_t TimeMeasurementSystem::getDataInArray(uint8_t *dataBuffer)
{
	uint8_t dataToReturn[TIME_MEASURE_OBJECTDATAVOLUME];

	for(uint8_t i = 0; i < TIME_MEASURE_OBJECTDATAVOLUME; i+=2)
	{
		dataToReturn[i] = elapsedTime[i/2] >> 8;
		dataToReturn[i+1] = elapsedTime[i/2] & 0xFF;
		elapsedTime[i/2] = 0;
	}

	std::copy_n(dataToReturn, TIME_MEASURE_OBJECTDATAVOLUME, dataBuffer);

	return TIME_MEASURE_OBJECTDATAVOLUME;
}

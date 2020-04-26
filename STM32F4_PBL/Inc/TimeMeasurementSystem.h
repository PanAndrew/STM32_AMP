/*
 * TimeMeasurementSystem.h
 *
 *  Created on: Mar 31, 2020
 *      Author: piotrek
 */

#ifndef TIMEMEASUREMENTSYSTEM_H_
#define TIMEMEASUREMENTSYSTEM_H_

#include "stdint.h"
#include "main.h"
#include <algorithm>

#define TIME_MEASURE_OBJECTS 6
#define TIME_MEASURE_OBJECTDATAVOLUME TIME_MEASURE_OBJECTS * 2

class TimeMeasurementSystem {

	uint16_t timestamps[TIME_MEASURE_OBJECTS] = {};
	uint16_t elapsedTime[TIME_MEASURE_OBJECTS] = {};

	TIM_HandleTypeDef *timerHandle;

public:
	TimeMeasurementSystem(TIM_HandleTypeDef *htim);
	virtual ~TimeMeasurementSystem();

	void takeTS(uint8_t choosenCell);
	void calculateElapsedTime(uint8_t choosenCell);

	uint8_t getDataInArray(uint8_t *dataBuffer);
};

#endif /* TIMEMEASUREMENTSYSTEM_H_ */

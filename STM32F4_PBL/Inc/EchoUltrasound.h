/*
 * EchoUltrasound.h
 *
 *  Created on: Mar 16, 2021
 *      Author: piotrek
 */

#ifndef ECHOULTRASOUND_H_
#define ECHOULTRASOUND_H_

#define ECHO_ULTRASOUND_OBJECTDATAVOLUME 4
#define SPEED_OF_SOUND 0.034 // cm/us

#include "stdint.h"
#include "main.h"
#include <algorithm>

class EchoUltrasound {

private:
	TIM_HandleTypeDef *triggerTimerHandle;
	uint8_t trigTimChannel;
	TIM_HandleTypeDef *counterTimerHandle;

	union floatUnion
	{
		float floatValue;
		uint8_t floatArr[sizeof(float)];
	};

	floatUnion distance;
	uint32_t lastTimeStamp;

	enum measureState
	{
		Finished,
		Running
	};

	enum echoState
	{
		Leading,
		Falling
	};

	measureState mstate;
	echoState desiredState;
	bool updated = false;

public:
	EchoUltrasound(TIM_HandleTypeDef *trigHTim, uint8_t trigTimchannel, TIM_HandleTypeDef *countHTim);
	virtual ~EchoUltrasound();

	void generateTriggerImpulse();
	void takeTimeStamp();
	void calculateDistance();

	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* ECHOULTRASOUND_H_ */

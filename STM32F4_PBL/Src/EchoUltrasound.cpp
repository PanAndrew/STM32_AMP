/*
 * EchoUltrasound.cpp
 *
 *  Created on: Mar 16, 2021
 *      Author: piotrek
 */

#include "EchoUltrasound.h"

EchoUltrasound::EchoUltrasound(TIM_HandleTypeDef *trigHTim, uint8_t trigTimCh, TIM_HandleTypeDef *countHTim):
								triggerTimerHandle(trigHTim), trigTimChannel(trigTimCh), counterTimerHandle(countHTim)
{
}

EchoUltrasound::~EchoUltrasound() {
	// TODO Auto-generated destructor stub
}

void EchoUltrasound::generateTriggerImpulse()
{
	__HAL_TIM_SET_COMPARE(triggerTimerHandle, trigTimChannel, triggerTimerHandle->Instance->ARR);
	mstate = Running;
	desiredState = Leading;
}

void EchoUltrasound::takeTimeStamp()
{
	if(mstate)
	{
		if(desiredState == Leading)
		{
			lastTimeStamp = counterTimerHandle->Instance->CNT;
			desiredState = Falling;
		}
//		else
//		{
//			if(currentState == Falling)
//			{
//				uint16_t diff = timerHandle->Instance->CNT - lastTimeStamp;
//
//				if(diff >= 0)
//				{
//					lastTimeStamp += diff;
//				}
//				else
//				{
//					lastTimeStamp += diff + timerHandle->Instance->ARR + 1;
//				}
//
//				mstate = Finished;
//
//			}
//		}
	}

}

void EchoUltrasound::calculateDistance()
{
	if(mstate)
	{
		if(desiredState == Falling)
		{
			uint16_t diff = counterTimerHandle->Instance->CNT - lastTimeStamp;

			if(diff >= 0)
			{
				distance.floatValue = (diff * SPEED_OF_SOUND) / 2.0;
			}
			else
			{
				distance.floatValue = ((diff + counterTimerHandle->Instance->ARR + 1) * SPEED_OF_SOUND) / 2.0;
			}

			mstate = Finished;
			updated = true;
		}
	}
}

uint8_t EchoUltrasound::getDataInArray(uint8_t* dataBuffer)
{
	if(updated)
	{
		updated = false;

		uint8_t dataToReturn[ECHO_ULTRASOUND_OBJECTDATAVOLUME];

		dataToReturn[0] = distance.floatArr[3];
		dataToReturn[1] = distance.floatArr[2];
		dataToReturn[2] = distance.floatArr[1];
		dataToReturn[3] = distance.floatArr[0];

		std::copy_n(dataToReturn, ECHO_ULTRASOUND_OBJECTDATAVOLUME, dataBuffer);

		return ECHO_ULTRASOUND_OBJECTDATAVOLUME;
	}
	else
	{
		return 0;
	}
}

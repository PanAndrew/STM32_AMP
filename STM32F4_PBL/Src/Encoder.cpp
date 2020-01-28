/*
 * Encoder.cpp
 *
 *  Created on: May 14, 2019
 *      Author: piotrek
 */

#include <Encoder.h>

Encoder::Encoder(): speedBuffer(DEFAULT_NUM_OF_CHECKS)
{
}

Encoder::~Encoder() {
	// TODO Auto-generated destructor stub
}

void Encoder::initialize(TIM_HandleTypeDef* htim)
{
	this->htim = htim;
}

bool Encoder::checkIfMoveMade()
{
	presentTimRegisterValue = htim->Instance->CNT;
	diff = presentTimRegisterValue - lastTimRegisterValue;

	if(abs(diff) > 0)
	{
		presentTimeStamp = HAL_GetTick();
		elapsedTime = presentTimeStamp - lastTimeStamp;
		lastTimeStamp += elapsedTime;
		lastTimRegisterValue = presentTimRegisterValue;
		return true;
	}
	else
	{
		return false;
	}
}

bool Encoder::checkIfOverflow()
{
	if(abs(diff) > HALF_OF_TIM_ARR)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Encoder::calcSpeed()
{
	tempSpeed.floatVal = ((abs(diff) * EXTERN_WHEEL_RATIO) / 10.0) / ((elapsedTime) / 1000.0);

	speedBuffer.put(tempSpeed);

	for(uint8_t i = 0; i < speedBuffer.size(); i++)
	{
		tempSpeed.floatVal += speedBuffer.getOfIndex(i).floatVal;
	}

	speed.floatVal = tempSpeed.floatVal / speedBuffer.size();
	distance.floatVal = (abs(diff) * EXTERN_WHEEL_RATIO) / 10.0;
}

void Encoder::encoderIteration()
{
	if(checkIfMoveMade())
	{
		if(diff > 0)
		{
			if(checkIfOverflow())
			{
				encoderState = backRun;
			}
			else
			{
				encoderState = forwardRun;
			}
		}
		else
		{
			if(checkIfOverflow())
			{
				encoderState = forwardRun;
			}
			else
			{
				encoderState = backRun;
			}
		}

		calcSpeed();
		numberOfGoOnChecks = DEFAULT_NUM_OF_CHECKS;
	}
	else
	{
		numberOfGoOnChecks--;

		tempSpeed.floatVal = 0;
		speedBuffer.put(tempSpeed);

		distance.floatVal = 0;

		if(!numberOfGoOnChecks)
		{
			speed.floatVal = 0;
			encoderState = idle_stat;
			numberOfGoOnChecks = DEFAULT_NUM_OF_CHECKS;
			lastTimeStamp = HAL_GetTick();
			speedBuffer.reset();
		}
	}
}

uint8_t Encoder::getDataInArray(uint8_t* dataBuffer)
{
	uint8_t dataToReturn[ENCODER_OBJECTDATAVOLUME];

	dataToReturn[0] = encoderState;
	dataToReturn[1] = speed.arrVal[3];
	dataToReturn[2] = speed.arrVal[2];
	dataToReturn[3] = speed.arrVal[1];
	dataToReturn[4] = speed.arrVal[0];
	dataToReturn[5] = distance.arrVal[3];
	dataToReturn[6] = distance.arrVal[2];
	dataToReturn[7] = distance.arrVal[1];
	dataToReturn[8] = distance.arrVal[0];


	std::copy_n(dataToReturn, ENCODER_OBJECTDATAVOLUME, dataBuffer);

	return ENCODER_OBJECTDATAVOLUME;
}

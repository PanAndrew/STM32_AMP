/*
 * UltrasoundModule.cpp
 *
 *  Created on: Apr 26, 2020
 *      Author: piotrek
 */

#include "UltrasoundModule.h"

UltrasoundModule::UltrasoundModule() {
	// TODO Auto-generated constructor stub

}

UltrasoundModule::~UltrasoundModule() {
	// TODO Auto-generated destructor stub
}

bool UltrasoundModule::calculateAndMatchCRC(uint8_t *dataBuffer)
{
	uint8_t checkSum = 0;
	for (uint8_t i = 0; i < 3; i++)
	{
		checkSum += dataBuffer[i];
	}

	return checkSum == dataBuffer[3];
}

void UltrasoundModule::encode(uint8_t *dataBuffer, uint16_t bufferSize)
{
	uint16_t iter = 0;
	uint16_t remainedDataSize = bufferSize;

	while(remainedDataSize)
	{
		if(remainedDataSize > ULTRASOUND_BYTES_TO_ENCODE - 1)
		{
			if(dataBuffer[iter] == 0x22)
			{
				if(calculateAndMatchCRC(&dataBuffer[iter]))
				{
					distance = (dataBuffer[iter + 1] << 8) + dataBuffer[iter + 2];
					updated = true;
				}

				iter += ULTRASOUND_BYTES_TO_ENCODE;
			}
			else
			{
				iter++;
			}
		}
		else
		{
			remainedDataSize = 0;
			continue;
		}

		remainedDataSize = bufferSize - iter;
	}
}

uint16_t UltrasoundModule::getDistance()
{
	return distance;
}

bool UltrasoundModule::isUpdated()
{
	return updated;
}

uint8_t UltrasoundModule::getDataInArray(uint8_t *dataBuffer)
{
	updated = false;

	uint8_t dataToReturn[ULTRASOUND_OBJECTDATAVOLUME];

	dataToReturn[0] = distance >> 8;
	dataToReturn[1] = distance & 0xFF;

	std::copy_n(dataToReturn, ULTRASOUND_OBJECTDATAVOLUME, dataBuffer);

	return ULTRASOUND_OBJECTDATAVOLUME;
}

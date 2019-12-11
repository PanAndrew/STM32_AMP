/*
 * MiniLidar.cpp
 *
 *  Created on: Dec 11, 2019
 *      Author: piotrek
 */

#include "MiniLidar.h"

MiniLidar::MiniLidar() {
	// TODO Auto-generated constructor stub

}

MiniLidar::~MiniLidar() {
	// TODO Auto-generated destructor stub
}

bool MiniLidar::calculateAndMatchCRC(uint8_t* dataBuffer)
{
	uint8_t checkSum = 0;
	for (uint8_t i = 0; i < 8; i++)
	{
		checkSum += dataBuffer[i];
	}

	return checkSum == dataBuffer[8];
}

void MiniLidar::encode(uint8_t* dataBuffer, uint16_t bufferSize)
{
	uint16_t iter = 0;
	uint16_t remainedDataSize = bufferSize;

	while(remainedDataSize)
	{
		if(remainedDataSize > BYTES_TO_ENCODE - 1)
		{
			if(dataBuffer[iter] == 0x59 && dataBuffer[iter + 1] == 0x59)
			{
				if(calculateAndMatchCRC(&dataBuffer[iter]))
				{
					distance = (dataBuffer[iter + 3] << 8) + dataBuffer[iter + 2];
					signalStrength = (dataBuffer[iter + 5] << 8) + dataBuffer[iter + 4];
					lidarMode = dataBuffer[iter + 6];
					updated = true;
				}

				iter += BYTES_TO_ENCODE;
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

uint16_t MiniLidar::getDistance()
{
	return distance;
}

uint16_t MiniLidar::getSignalStrength()
{
	return signalStrength;
}

uint8_t MiniLidar::getLidarMode()
{
	return lidarMode;
}

bool MiniLidar::isUpdated()
{
	return updated;
}

uint8_t MiniLidar::getDataInArray(uint8_t* dataBuffer)
{
	updated = false;

	uint8_t dataToReturn[MINI_LIDAR_OBJECTDATAVOLUME];

	dataToReturn[0] = distance >> 8;
	dataToReturn[1] = distance & 0xFF;
	dataToReturn[2] = signalStrength >> 8;
	dataToReturn[3] = signalStrength & 0xFF;
	dataToReturn[4] = lidarMode;

	std::copy_n(dataToReturn, MINI_LIDAR_OBJECTDATAVOLUME, dataBuffer);

	return MINI_LIDAR_OBJECTDATAVOLUME;
}

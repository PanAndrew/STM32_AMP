/*
 * TimeSync.cpp
 *
 *  Created on: Jan 17, 2020
 *      Author: piotrek
 */

#include "TimeSync.h"

TimeSync::TimeSync() {
	// TODO Auto-generated constructor stub

}

TimeSync::~TimeSync() {
	// TODO Auto-generated destructor stub
}

uint8_t TimeSync::getDataInArray(uint8_t* dataBuffer)
{
	uint8_t dataToReturn[TIME_SYNC_OBJECTDATAVOLUME];

	timeStamp = HAL_GetTick();

	dataToReturn[0] = timeStamp >> 24;
	dataToReturn[1] = timeStamp >> 16;
	dataToReturn[2] = timeStamp >> 8;
	dataToReturn[3] = timeStamp & 0xFF;

	std::copy_n(dataToReturn, TIME_SYNC_OBJECTDATAVOLUME, dataBuffer);

	return TIME_SYNC_OBJECTDATAVOLUME;
}

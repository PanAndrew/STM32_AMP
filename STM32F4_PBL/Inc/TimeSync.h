/*
 * TimeSync.h
 *
 *  Created on: Jan 17, 2020
 *      Author: piotrek
 */

#ifndef TIMESYNC_H_
#define TIMESYNC_H_

#define TIME_SYNC_OBJECTDATAVOLUME 4

#include "main.h"
#include <algorithm>
#include "stdint.h"

class TimeSync {

	uint32_t timeStamp;

public:
	TimeSync();
	virtual ~TimeSync();

	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* TIMESYNC_H_ */

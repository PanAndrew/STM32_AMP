/*
 * IMUData.h
 *
 *  Created on: May 26, 2019
 *      Author: piotrek
 */

#ifndef IMUDATA_H_
#define IMUDATA_H_

#include "stdint.h"
#include <array>
#include <algorithm>

#define DATASIZE 6
#define IMU_OBJECTDATAVOLUME (DATASIZE + sizeof(uint32_t))

class IMUData {

	uint8_t data[DATASIZE];
	uint32_t timestamp;
	uint8_t size;

	uint8_t getByte(uint32_t number, uint8_t part);

public:
	IMUData();
	IMUData(const uint8_t *rawData, uint32_t time, const uint8_t &dataSize);
	virtual ~IMUData();

	uint8_t* getData();
	uint32_t getTimestamp();
	uint8_t getDataInArray(uint8_t* dataBuffer);
	uint8_t getSize();
	uint8_t getObjectDataVolume();
};

#endif /* IMUDATA_H_ */

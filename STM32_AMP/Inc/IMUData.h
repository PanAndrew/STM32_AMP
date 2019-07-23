/*
 * IMUData.h
 *
 *  Created on: May 26, 2019
 *      Author: piotrek
 */

#ifndef IMUDATA_H_
#define IMUDATA_H_

#include "stdint.h"
#include <algorithm>
#include <memory>

class IMUData {

	std::shared_ptr<uint8_t[]> data;
	uint32_t timestamp;
	uint8_t size;

	uint8_t getByte(uint32_t number, uint8_t part);

public:
	IMUData();
	IMUData(const IMUData &imuData);
	IMUData(const uint8_t *rawData, uint32_t time, const uint8_t &dataSize);
	virtual ~IMUData();

	std::shared_ptr<uint8_t[]> getData() const;
	uint32_t getTimestamp() const;
	uint8_t getDataInArray(std::shared_ptr<uint8_t[]> *dataBuffer);
	uint8_t getSize() const;
	uint8_t getObjectDataVolume();
};

#endif /* IMUDATA_H_ */

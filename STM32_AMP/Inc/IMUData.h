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

	std::unique_ptr<uint8_t[]> data;
	uint32_t timestamp;
	uint16_t size;

	uint8_t getByte(uint32_t number, uint8_t part);

public:
	IMUData();
	IMUData(const IMUData &imuData);
	IMUData& operator=(const IMUData &imuData);
//	IMUData& operator=(IMUData&& imuData);
	IMUData(const uint8_t *rawData, const uint32_t &time, const uint16_t &dataSize);
	virtual ~IMUData();

	const uint8_t* getData() const;
	uint32_t getTimestamp() const;
	uint8_t *getDataInArray();
	uint16_t getSize();

};

#endif /* IMUDATA_H_ */

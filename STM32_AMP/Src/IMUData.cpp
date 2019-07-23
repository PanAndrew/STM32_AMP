/*
 * IMUData.cpp
 *
 *  Created on: May 26, 2019
 *      Author: piotrek
 */

#include "IMUData.h"

IMUData::IMUData()
{
}

IMUData::IMUData(const IMUData &imuData):
		data(imuData.getData()), timestamp(imuData.getTimestamp()), size(imuData.getSize())
{
}

IMUData::IMUData(const uint8_t *rawData, uint32_t time, const uint8_t &dataSize):
		data(new uint8_t[dataSize]), timestamp(time), size(dataSize)
{
	std::copy(rawData, rawData + dataSize, data.get());
}

IMUData::~IMUData() {
	// TODO Auto-generated destructor stub
}

std::shared_ptr<uint8_t[]> IMUData::getData() const
{
	return data;
}

uint32_t IMUData::getTimestamp() const
{
	return timestamp;
}

uint8_t IMUData::getByte(uint32_t number, uint8_t part)
{
	return (number >> (8 * part)) & 0xFF;
}

uint8_t IMUData::getDataInArray(std::shared_ptr<uint8_t[]> *dataBuffer)
{
	std::shared_ptr<uint8_t[]> dataToReturn = std::shared_ptr<uint8_t[]>(new uint8_t[10]);

	std::copy(data.get(), data.get() + 6, dataToReturn.get());

	for(uint8_t i = 0; i < 4; i++)
	{
		dataToReturn.get()[6 + i] = getByte(timestamp, i);
	}

	dataBuffer = &dataToReturn;

	return size + sizeof(timestamp);
}

uint8_t IMUData::getSize() const
{
	return size;
}

uint8_t IMUData::getObjectDataVolume()
{
	return size + sizeof(timestamp);
}

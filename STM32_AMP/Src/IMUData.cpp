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
		data(std::move(imuData.getData())), timestamp(imuData.getTimestamp()), size(imuData.getSize())
{
}

IMUData& IMUData::operator=(const IMUData &imuData)
{
	data = std::move(imuData.getData());
	timestamp = imuData.getTimestamp();
	size = imuData.getSize();
	return *this;
}

IMUData::IMUData(const uint8_t *rawData, uint32_t time, const uint8_t &dataSize):
		data(new uint8_t[dataSize], [](uint8_t *ptr) { delete[] ptr;}), timestamp(time), size(dataSize)
{
	std::copy_n(rawData, dataSize, data.get());
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

uint8_t IMUData::getDataInArray(std::shared_ptr<uint8_t[]> &dataBuffer)
{
	std::shared_ptr<uint8_t[]> dataToReturn(new uint8_t[size + sizeof timestamp], [](uint8_t *ptr) { delete[] ptr;});

	dataToReturn = std::move(data);

	for(uint8_t i = 0; i < sizeof timestamp ; i++)
	{
		dataToReturn.get()[6 + i] = getByte(timestamp, sizeof timestamp  - 1 - i);
	}

	dataBuffer = std::move(dataToReturn);
	dataToReturn.reset();
	return size + sizeof timestamp;
}

uint8_t IMUData::getSize() const
{
	return size;
}

uint8_t IMUData::getObjectDataVolume()
{
	return size + sizeof timestamp;
}

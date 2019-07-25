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
		timestamp(imuData.getTimestamp()), size(imuData.getSize())
{
	std::move(imuData.getData().get(), imuData.getData().get() + size, data.get());
}

IMUData& IMUData::operator=(const IMUData &imuData)
{
	data = std::make_unique<uint8_t[]>(imuData.getSize());
	data = std::move(imuData.getData());
	timestamp = imuData.getTimestamp();
	size = imuData.getSize();
	return *this;
}

IMUData::IMUData(const uint8_t *rawData, uint32_t time, const uint8_t &dataSize):
		timestamp(time), size(dataSize)
{
	data = std::make_unique<uint8_t[]>(dataSize);
	std::copy_n(rawData, dataSize, data.get());
}

IMUData::~IMUData() {
	// TODO Auto-generated destructor stub
}

std::unique_ptr<uint8_t[]> IMUData::getData() const
{
	auto uniqueToReturn = std::make_unique<uint8_t[]>(size * sizeof timestamp);
	std::copy(data.get(), data.get() + size, uniqueToReturn.get());
	return uniqueToReturn;
}

uint32_t IMUData::getTimestamp() const
{
	return timestamp;
}

uint8_t IMUData::getByte(uint32_t number, uint8_t part)
{
	return (number >> (8 * part)) & 0xFF;
}

uint8_t IMUData::getDataInArray(std::unique_ptr<uint8_t[]> &dataBuffer)
{
	std::unique_ptr<uint8_t[]> dataToReturn = std::make_unique<uint8_t[]>(size + sizeof timestamp);

	dataToReturn = std::move(data);

	for(uint8_t i = 0; i < sizeof(timestamp); i++)
	{
		dataToReturn.get()[6 + i] = getByte(timestamp, sizeof(timestamp) - 1 - i);
	}

	dataBuffer = std::move(dataToReturn);

	return size + sizeof timestamp;
}

uint8_t IMUData::getSize() const
{
	return size;
}

uint8_t IMUData::getObjectDataVolume()
{
	return size + sizeof(timestamp);
}

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

IMUData::IMUData(const uint8_t *rawData, uint32_t time, const uint8_t &dataSize):
		timestamp(time), size(dataSize)
{
	std::copy_n(rawData, dataSize, data);
}

IMUData::~IMUData()
{
}

uint8_t* IMUData::getData()
{
	return data;
}

uint32_t IMUData::getTimestamp()
{
	return timestamp;
}

uint8_t IMUData::getByte(uint32_t number, uint8_t part)
{
	return (number >> (8 * part)) & 0xFF;
}

uint8_t IMUData::getDataInArray(uint8_t* dataBuffer)
{
	uint8_t dataToReturn[IMU_OBJECTDATAVOLUME];

	std::copy_n(data, DATASIZE, dataToReturn);

	for(uint8_t i = 0; i < sizeof timestamp ; i++)
	{
		dataToReturn[6 + i] = getByte(timestamp, sizeof timestamp  - 1 - i);
	}

	std::copy_n(dataToReturn, IMU_OBJECTDATAVOLUME, dataBuffer);

	return IMU_OBJECTDATAVOLUME;
}

uint8_t IMUData::getSize()
{
	return size;
}

uint8_t IMUData::getObjectDataVolume()
{
	return IMU_OBJECTDATAVOLUME;
}

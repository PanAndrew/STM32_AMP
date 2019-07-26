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

//IMUData::IMUData(const IMUData &imuData):
//		data(*(imuData.getData())), timestamp(imuData.getTimestamp()), size(imuData.getSize())
//{
//}

IMUData::IMUData(const uint8_t *rawData, uint32_t time, const uint8_t &dataSize):
		timestamp(time), size(dataSize)
{
	std::copy_n(rawData, dataSize, data.data());
}

IMUData::~IMUData()
{
}

std::array<uint8_t, DATASIZE>* IMUData::getData()
{
	return &data;
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
	std::array<uint8_t, OBJECTDATAVOLUME> dataToReturn;

	std::copy_n(data.data(), data.size(), dataToReturn.data());

	for(uint8_t i = 0; i < sizeof timestamp ; i++)
	{
		dataToReturn[6 + i] = getByte(timestamp, sizeof timestamp  - 1 - i);
	}

	std::copy_n(dataToReturn.data(), dataToReturn.size(), dataBuffer);

	return OBJECTDATAVOLUME;
}

uint8_t IMUData::getSize()
{
	return size;
}

uint8_t IMUData::getObjectDataVolume()
{
	return OBJECTDATAVOLUME;
}

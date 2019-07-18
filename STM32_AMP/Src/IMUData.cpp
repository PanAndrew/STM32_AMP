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
		data(std::move(imuData.data.get())), timestamp(imuData.timestamp), size(imuData.size) // .get() or .release();
{
}

IMUData& IMUData::operator=(const IMUData &imuData)
{
   data.reset(std::move(imuData.data.get()));
   return *this;
}

//IMUData& IMUData::operator=(IMUData&& imuData)
//{
//   data = std::move(imuData.data);
//   return *this;
//}

IMUData::IMUData(const uint8_t *rawData, const uint32_t &time, const uint16_t &dataSize):
		data(std::unique_ptr<uint8_t[]>(new uint8_t[dataSize])), timestamp(time), size(dataSize)
{
	std::copy(rawData, rawData + dataSize, data.get());
}

IMUData::~IMUData() {
	// TODO Auto-generated destructor stub
}

const uint8_t* IMUData::getData() const
{
	return data.get();
}

uint32_t IMUData::getTimestamp() const
{
	return timestamp;
}

uint8_t IMUData::getByte(uint32_t number, uint8_t part)
{
	return (number >> (8 * part)) & 0xFF;
}

uint8_t* IMUData::getDataInArray()
{
	std::unique_ptr<uint8_t[]> dataToReturn = std::unique_ptr<uint8_t[]>(new uint8_t[10]);

	std::copy(data.get(), data.get() + 6, dataToReturn.get());

	for(uint8_t i = 0; i < 4; i++)
	{
		dataToReturn.get()[6 + i] = getByte(timestamp, i);
	}

	return dataToReturn.get();
}

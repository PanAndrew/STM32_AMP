/*
 * DataManagement.cpp
 *
 *  Created on: Jul 27, 2019
 *      Author: piotrek
 */

#include "DataManagement.h"
#include <algorithm>
#include <memory>

DataManagement::DataManagement() {
	// TODO Auto-generated constructor stub

}

DataManagement::~DataManagement() {
	// TODO Auto-generated destructor stub
}

void DataManagement::configure(std::map<uint8_t, DataPtrVolumePair>* ptrMap)
{
	dataPtrMap = ptrMap;

	std::unique_ptr<uint8_t[]> tempArray = std::make_unique<uint8_t[]>(dataPtrMap->size());

	uint8_t i = 0;
	for(auto const &iter : *dataPtrMap)
	{
		tempArray[i] = iter.first;
		i++;
	}

	confRefArray(tempArray.get(), dataPtrMap->size());
}

void DataManagement::confRefArray(uint8_t* confArray, uint8_t numbOfElem)
{
	refDataArray.clear();
	refDataArray.reserve(numbOfElem);

	for(uint8_t i = 0; i < numbOfElem; i++)
	{
		if(dataPtrMap->find(confArray[i]) != dataPtrMap->end())
		{
			refDataArray.push_back(confArray[i]);
		}
	}

	maxDataSize = calcRefArrDataSize();
}

uint16_t DataManagement::calcRefArrDataSize()
{
	uint16_t size = 0;
	DataPtrVolumePair* tempObj;

	for(uint8_t indexID : refDataArray)
	{
		tempObj = &(dataPtrMap->at(indexID));
		size += tempObj->getNumberOfElements() * tempObj->getSizeOfElement();
	}

	return size + (2 * refDataArray.size());
}

uint8_t DataManagement::getConfigInArray(uint8_t* dataBuffer)
{
	std::copy_n(refDataArray.data(), refDataArray.size(), dataBuffer);

	return refDataArray.size();
}

uint16_t DataManagement::getMaxDataSize()
{
	return maxDataSize;
}

void DataManagement::incrementFrameErrorCnt()
{
	frameErrors++;
}

uint8_t DataManagement::getPacketsInfo(uint8_t *dataBuffer)
{
	uint8_t dataToReturn[PACKET_NUM_OBJDATAVOLUME];

	dataToReturn[0] = packetNumber >> 24;
	dataToReturn[1] = packetNumber >> 16;
	dataToReturn[2] = packetNumber >> 8;
	dataToReturn[3] = packetNumber & 0xFF;
	dataToReturn[4] = frameErrors >> 24;
	dataToReturn[5] = frameErrors >> 16;
	dataToReturn[6] = frameErrors >> 8;
	dataToReturn[7] = frameErrors & 0xFF;

	std::copy_n(dataToReturn, PACKET_NUM_OBJDATAVOLUME, dataBuffer);

	return PACKET_NUM_OBJDATAVOLUME;
}

uint16_t DataManagement::getCollectedData(uint8_t* globalBuffer)
{
	std::unique_ptr<uint8_t[]> dataBuffer = std::make_unique<uint8_t[]>(maxDataSize);
	DataPtrVolumePair* tempObj;
	uint16_t dataSize = 0;
	uint16_t returnedSize = 0;

	for(auto indexID : refDataArray)
	{
		dataBuffer[dataSize] = indexID;
		dataSize++;

		tempObj = &(dataPtrMap->at(indexID));
		returnedSize = tempObj->getFunction()(&dataBuffer[++dataSize]);

		if(!returnedSize)
		{
			dataSize -= 2;
			continue;
		}

		dataBuffer[dataSize - 1] = returnedSize / tempObj->getSizeOfElement();

		dataSize += returnedSize;
	}

	std::copy_n(dataBuffer.get(), dataSize, globalBuffer);

	packetNumber++;
	return dataSize;
}

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
	for(auto iter : *dataPtrMap)
	{
		tempArray[i] = iter.first;
		i++;
	}

	confRefArray(tempArray.get());
}

void DataManagement::confRefArray(uint8_t* confArray)
{
	refDataArray.clear();
	refDataArray.reserve(sizeof confArray / sizeof(uint8_t));
	std::copy_n(confArray, sizeof confArray / sizeof(uint8_t), refDataArray.data());

	maxDataSize = calcRefArrDataSize();
}

uint16_t DataManagement::calcRefArrDataSize()
{
	uint16_t size = 0;
	DataPtrVolumePair* tempObj;

	for(auto iter : refDataArray)
	{
		tempObj = &dataPtrMap->at(iter);
		size += tempObj->getNumberOfElements() * tempObj->getSizeOfElement();
	}

	return size;
}

uint8_t DataManagement::getConfigInArray(uint8_t* dataBuffer)
{
	std::copy_n(refDataArray.data(), refDataArray.size(), dataBuffer);

	return refDataArray.size();
}

void DataManagement::sendData()
{
	std::unique_ptr<uint8_t[]> dataBuffer = std::make_unique<uint8_t[]>(maxDataSize);
	uint16_t dataSize = 0;

	for(auto iter : refDataArray)
	{
		dataBuffer[dataSize] = iter;
		dataSize++;
		dataSize += dataPtrMap->at(iter).getFunction()(dataBuffer.get() + dataSize);
	}
}


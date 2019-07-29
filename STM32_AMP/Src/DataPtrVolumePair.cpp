/*
 * dataPtrVolumePair.cpp
 *
 *  Created on: Jul 27, 2019
 *      Author: piotrek
 */

#include "DataPtrVolumePair.h"

DataPtrVolumePair::DataPtrVolumePair(uint8_t dataSize, std::function<uint8_t(uint8_t*)> ptrToFunction):
		size(dataSize), functionPtr(ptrToFunction)
{
}

//DataPtrVolumePair::DataPtrVolumePair(std::pair <uint8_t, std::function<uint8_t(uint8_t*)>> pair):
//		size(pair.first), functionPtr(pair.second)
//{
//}

//DataPtrVolumePair::DataPtrVolumePair(std::pair <uint8_t, uint8_t (*)(uint8_t*)> pair):
//		size(pair.first), functionPtr(pair.second)
//{
//}

//DataPtrVolumePair::DataPtrVolumePair(uint8_t dataSize, uint8_t (*ptrToFunction)(uint8_t *)) :
//		size(dataSize), functionPtr(ptrToFunction)
//{
//}

DataPtrVolumePair::~DataPtrVolumePair()
{
}

uint8_t DataPtrVolumePair::getSize()
{
	return size;
}

std::function<uint8_t(uint8_t*)> DataPtrVolumePair::getFunction()
{
	return functionPtr;
}

//uint8_t (*DataPtrVolumePair::getFunction())(uint8_t *)
//{
//	return functionPtr;
//}

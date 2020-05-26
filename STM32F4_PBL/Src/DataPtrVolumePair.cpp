/*
 * dataPtrVolumePair.cpp
 *
 *  Created on: Jul 27, 2019
 *      Author: piotrek
 */

#include "DataPtrVolumePair.h"

DataPtrVolumePair::DataPtrVolumePair(uint8_t numbOfElem, uint8_t dataSize, std::function<uint16_t(uint8_t*)> ptrToFunction):
			numberOfElements(numbOfElem), sizeOfElement(dataSize), functionPtr(ptrToFunction)
{
}

DataPtrVolumePair::~DataPtrVolumePair()
{
}

uint8_t DataPtrVolumePair::getNumberOfElements()
{
	return numberOfElements;
}

uint8_t DataPtrVolumePair::getSizeOfElement()
{
	return sizeOfElement;
}

std::function<uint16_t(uint8_t*)> DataPtrVolumePair::getFunction()
{
	return functionPtr;
}

/*
 * dataPtrVolumePair.h
 *
 *  Created on: Jul 27, 2019
 *      Author: piotrek
 */

#ifndef DATAPTRVOLUMEPAIR_H_
#define DATAPTRVOLUMEPAIR_H_

#include "stdint.h"
#include <utility>
#include <functional>

class DataPtrVolumePair {

	uint8_t numberOfElements;
	uint8_t sizeOfElement;
	std::function<uint16_t(uint8_t*)> functionPtr;

public:
	DataPtrVolumePair(uint8_t numbOfElem, uint8_t dataSize, std::function<uint16_t(uint8_t*)> ptrToFunction);
	virtual ~DataPtrVolumePair();

	uint8_t getNumberOfElements();
	uint8_t getSizeOfElement();
	std::function<uint16_t(uint8_t*)> getFunction();
};

#endif /* DATAPTRVOLUMEPAIR_H_ */

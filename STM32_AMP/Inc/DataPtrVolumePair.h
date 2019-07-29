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

	uint8_t size;
	std::function<uint8_t(uint8_t*)> functionPtr;
//	uint8_t (*functionPtr)(uint8_t *);

public:
	DataPtrVolumePair(uint8_t , std::function<uint8_t(uint8_t*)>);
//	DataPtrVolumePair(std::pair <uint8_t, std::function<uint8_t(uint8_t*)>> pair);
//	DataPtrVolumePair(std::pair <uint8_t, uint8_t (*)(uint8_t*)> pair);
//	DataPtrVolumePair(uint8_t id, uint8_t (*ptrToFunction)(uint8_t *));
	virtual ~DataPtrVolumePair();

	uint8_t getSize();
	std::function<uint8_t(uint8_t*)> getFunction();
//	uint8_t (*getFunction())(uint8_t *);
};

#endif /* DATAPTRVOLUMEPAIR_H_ */

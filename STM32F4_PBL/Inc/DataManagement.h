/*
 * DataManagement.h
 *
 *  Created on: Jul 27, 2019
 *      Author: piotrek
 */

#ifndef DATAMANAGEMENT_H_
#define DATAMANAGEMENT_H_

#include <map>
#include <vector>
#include "DataPtrVolumePair.h"
#include <stdio.h>

#define PACKET_NUM_OBJDATAVOLUME 8

class DataManagement {

	std::map<uint8_t, DataPtrVolumePair>* dataPtrMap;
	std::vector<uint8_t> refDataArray;
	uint16_t maxDataSize;

	uint32_t packetNumber = 0;
	uint32_t frameErrors = 0;

	uint16_t calcRefArrDataSize();

public:
	DataManagement();
	virtual ~DataManagement();

	void configure(std::map<uint8_t, DataPtrVolumePair>* ptrMap);
	void confRefArray(uint8_t * confArray, uint8_t numbOfElem);
	uint8_t getConfigInArray(uint8_t* dataBuffer);
	uint16_t getCollectedData(uint8_t* globalBuffer);
	uint16_t getMaxDataSize();
	void incrementFrameErrorCnt();

	uint8_t getPacketsInfo(uint8_t* dataBuffer);
};

#endif /* DATAMANAGEMENT_H_ */

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

class DataManagement {

	std::map<uint8_t, DataPtrVolumePair>* dataPtrMap;
	std::vector<uint8_t> refDataArray;
	uint16_t maxDataSize;

	void confRefArray(uint8_t * confArray);
	uint16_t calcRefArrDataSize();

public:
	DataManagement();
	virtual ~DataManagement();

	void configure(std::map<uint8_t, DataPtrVolumePair>* ptrMap);
	uint8_t getConfigInArray(uint8_t* dataBuffer);
	void sendData();

};

#endif /* DATAMANAGEMENT_H_ */

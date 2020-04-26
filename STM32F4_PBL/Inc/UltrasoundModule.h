/*
 * UltrasoundModule.h
 *
 *  Created on: Apr 26, 2020
 *      Author: piotrek
 */

#ifndef ULTRASOUNDMODULE_H_
#define ULTRASOUNDMODULE_H_

#include "stdint.h"
#include <algorithm>

#define ULTRASOUND_BYTES_TO_ENCODE 4
#define ULTRASOUND_OBJECTDATAVOLUME 2

class UltrasoundModule {

	bool updated = false;
	uint16_t distance;

	bool calculateAndMatchCRC(uint8_t* dataBuffer);

public:
	UltrasoundModule();
	virtual ~UltrasoundModule();

	void encode(uint8_t* dataBuffer, uint16_t bufferSize);

	uint16_t getDistance();

	bool isUpdated();
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* ULTRASOUNDMODULE_H_ */

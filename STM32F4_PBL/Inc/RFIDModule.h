/*
 * RFIDModule.h
 *
 *  Created on: Mar 22, 2020
 *      Author: piotrek
 */

#ifndef RFIDMODULE_H_
#define RFIDMODULE_H_

#include "stdint.h"
#include <algorithm>

#define RFID_FRAME_START_BYTE 0x02
#define RFID_FRAME_END_BYTE 0x03
#define RFID_BYTES_TO_ENCODE 16
#define RFID_OBJECTDATAVOLUME 5

class RFIDModule {

	bool updated = false;
	uint8_t idCode[5];

	bool calculateAndMatchCRC(uint8_t* dataBuffer);
	void fromASCIIToByteArray(uint8_t* src, uint8_t* dst);

public:
	RFIDModule();
	virtual ~RFIDModule();

	void encode(uint8_t* dataBuffer, uint16_t bufferSize);
	bool isUpdated();
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* RFIDMODULE_H_ */

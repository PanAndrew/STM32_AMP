/*
 * RFIDModule.cpp
 *
 *  Created on: Mar 22, 2020
 *      Author: piotrek
 */

#include "RFIDModule.h"

RFIDModule::RFIDModule() {
	// TODO Auto-generated constructor stub

}

RFIDModule::~RFIDModule() {
	// TODO Auto-generated destructor stub
}

bool RFIDModule::calculateAndMatchCRC(uint8_t* dataBuffer)
{
	uint8_t checkSum = 0;
	for (uint8_t i = 0; i < 5; i++)
	{
		checkSum ^= dataBuffer[i];
	}

	if((checkSum == dataBuffer[5]) && (dataBuffer[8] == RFID_FRAME_END_BYTE))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void RFIDModule::fromASCIIToByteArray(uint8_t* src, uint8_t* dst)
{
	uint8_t iter = 0;

	for(uint8_t i = 0; i < 6; i++)
	{
		dst[i] = (src[iter] << 4) + src[iter + 1];
		iter += 2;
	}

	dst[6] = src[iter - 1];
	dst[7] = src[iter];
	dst[8] = src[iter + 1];
}

void RFIDModule::encode(uint8_t *dataBuffer, uint16_t bufferSize)
{
	uint16_t iter = 0;
	uint16_t remainedDataSize = bufferSize;

	while(remainedDataSize)
	{
		if(remainedDataSize > RFID_BYTES_TO_ENCODE - 1)
		{
			if(dataBuffer[iter] == RFID_FRAME_START_BYTE)
			{
				uint8_t transArray[9];

				fromASCIIToByteArray(&dataBuffer[iter + 1], transArray);

				if(calculateAndMatchCRC(transArray))
				{
					std::copy_n(transArray, RFID_OBJECTDATAVOLUME, idCode);
					updated = true;
				}

				iter += RFID_BYTES_TO_ENCODE;
			}
			else
			{
				iter++;
			}
		}
		else
		{
			remainedDataSize = 0;
			continue;
		}

		remainedDataSize = bufferSize - iter;
	}
}

bool RFIDModule::isUpdated()
{
	return updated;
}

uint8_t RFIDModule::getDataInArray(uint8_t *dataBuffer)
{
	updated = false;

	std::copy_n(idCode, RFID_OBJECTDATAVOLUME, dataBuffer);

	return RFID_OBJECTDATAVOLUME;
}

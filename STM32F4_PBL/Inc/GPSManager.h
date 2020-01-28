/*
 * GPSManager.h
 *
 *  Created on: Dec 9, 2019
 *      Author: piotrek
 */

#ifndef GPSMANAGER_H_
#define GPSMANAGER_H_

#include "stdint.h"
#include "UARTBuffer.h"
#include <memory>
#include "TinyGPS++.h"

#define GPS_UART_BUFFER_LENGTH 255
#define GPS_OBJECTDATAVOLUME 44

class GPSManager {

	UARTBuffer uartBuffer;
	TinyGPSPlus gps;

	union doubleUnion
	{
		double doubleValue;
		uint8_t doubleArr[sizeof(double)];
	};

	union floatUnion
	{
		float floatValue;
		uint8_t floatArr[sizeof(float)];
	};

	doubleUnion doubleUnionObj;
	floatUnion floatUnionObj;

public:
	GPSManager(DMA_HandleTypeDef* hdmaRxUart);
	virtual ~GPSManager();

	void processRxData();
	uint16_t getBufferLength();
	uint8_t* getDataBuffer();
	uint8_t getDataInArray(uint8_t* dataBuffer);

};

#endif /* GPSMANAGER_H_ */

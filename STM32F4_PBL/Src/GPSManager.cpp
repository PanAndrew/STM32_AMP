/*
 * GPSManager.cpp
 *
 *  Created on: Dec 9, 2019
 *      Author: piotrek
 */

#include "GPSManager.h"

GPSManager::GPSManager(DMA_HandleTypeDef* hdmaRxUart): uartBuffer(hdmaRxUart, GPS_UART_BUFFER_LENGTH)
{
}

GPSManager::~GPSManager() {
	// TODO Auto-generated destructor stub
}

void GPSManager::processRxData()
{
	std::unique_ptr<uint8_t[]> dataBuffer = std::make_unique<uint8_t[]>(uartBuffer.getBufferLength());
	uint16_t numOfReturnedElem;
	uint16_t iter = 0;

	uartBuffer.checkRxBuffer();
	numOfReturnedElem = uartBuffer.getData(dataBuffer.get());

	while(numOfReturnedElem--)
	{
		gps.encode(dataBuffer[iter++]);
	}
}

uint16_t GPSManager::getBufferLength()
{
	return uartBuffer.getBufferLength();
}

uint8_t* GPSManager::getDataBuffer()
{
	return uartBuffer.getDataBuffer();
}

uint8_t GPSManager::getDataInArray(uint8_t* dataBuffer)
{
	if(gps.time.isUpdated())
	{
		uint8_t dataToReturn[GPS_OBJECTDATAVOLUME];
		uint32_t tempValue;

		//Satellites
		dataToReturn[0] = gps.satellites.value();

		//HDOP
		floatUnionObj.floatValue = gps.hdop.hdop();
		dataToReturn[1] = floatUnionObj.floatArr[3];
		dataToReturn[2] = floatUnionObj.floatArr[2];
		dataToReturn[3] = floatUnionObj.floatArr[1];
		dataToReturn[4] = floatUnionObj.floatArr[0];

		//Latitude
		doubleUnionObj.doubleValue = gps.location.lat();
		dataToReturn[5] = doubleUnionObj.doubleArr[7];
		dataToReturn[6] = doubleUnionObj.doubleArr[6];
		dataToReturn[7] = doubleUnionObj.doubleArr[5];
		dataToReturn[8] = doubleUnionObj.doubleArr[4];
		dataToReturn[9] = doubleUnionObj.doubleArr[3];
		dataToReturn[10] = doubleUnionObj.doubleArr[2];
		dataToReturn[11] = doubleUnionObj.doubleArr[1];
		dataToReturn[12] = doubleUnionObj.doubleArr[0];

		//Longitude
		doubleUnionObj.doubleValue = gps.location.lng();
		dataToReturn[13] = doubleUnionObj.doubleArr[7];
		dataToReturn[14] = doubleUnionObj.doubleArr[6];
		dataToReturn[15] = doubleUnionObj.doubleArr[5];
		dataToReturn[16] = doubleUnionObj.doubleArr[4];
		dataToReturn[17] = doubleUnionObj.doubleArr[3];
		dataToReturn[18] = doubleUnionObj.doubleArr[2];
		dataToReturn[19] = doubleUnionObj.doubleArr[1];
		dataToReturn[20] = doubleUnionObj.doubleArr[0];

		//Info_Age
		tempValue = gps.location.age();
		dataToReturn[21] = tempValue >> 24;
		dataToReturn[22] = tempValue >> 16;
		dataToReturn[23] = tempValue >> 8;
		dataToReturn[24] = tempValue && 0xFF;

		//Data
		dataToReturn[25] = gps.date.day();
		dataToReturn[26] = gps.date.month();
		tempValue = gps.date.year();
		dataToReturn[27] = tempValue >> 8;
		dataToReturn[28] = tempValue && 0xFF;

		//Time
		dataToReturn[29] = gps.time.hour();
		dataToReturn[30] = gps.time.minute();
		dataToReturn[31] = gps.time.second();

		//Altitude - meters
		floatUnionObj.floatValue = gps.altitude.meters();
		dataToReturn[32] = floatUnionObj.floatArr[3];
		dataToReturn[33] = floatUnionObj.floatArr[2];
		dataToReturn[34] = floatUnionObj.floatArr[1];
		dataToReturn[35] = floatUnionObj.floatArr[0];

		//Course - deg
		floatUnionObj.floatValue = gps.course.deg();
		dataToReturn[36] = floatUnionObj.floatArr[3];
		dataToReturn[37] = floatUnionObj.floatArr[2];
		dataToReturn[38] = floatUnionObj.floatArr[1];
		dataToReturn[39] = floatUnionObj.floatArr[0];

		//Speed - km/h
		floatUnionObj.floatValue = gps.speed.kmph();
		dataToReturn[40] = floatUnionObj.floatArr[3];
		dataToReturn[41] = floatUnionObj.floatArr[2];
		dataToReturn[42] = floatUnionObj.floatArr[1];
		dataToReturn[43] = floatUnionObj.floatArr[0];

		std::copy_n(dataToReturn, GPS_OBJECTDATAVOLUME, dataBuffer);

		return GPS_OBJECTDATAVOLUME;
	}
	else
	{
		return 0;
	}
}

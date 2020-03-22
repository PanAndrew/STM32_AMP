/*
 * RFIDManager.cpp
 *
 *  Created on: Mar 22, 2020
 *      Author: piotrek
 */

#include "RFIDManager.h"

RFIDManager::RFIDManager(DMA_HandleTypeDef* hdmaRxUart) : uartBuffer(hdmaRxUart, MINI_RFID_UART_BUFFER_LENGTH)
{
}

RFIDManager::~RFIDManager()
{
}

void RFIDManager::processRxData()
{
	std::unique_ptr<uint8_t[]> dataBuffer = std::make_unique<uint8_t[]>(uartBuffer.getBufferLength());
	uint16_t numOfReturnedElem;

	uartBuffer.checkRxBuffer();
	numOfReturnedElem = uartBuffer.getData(dataBuffer.get());

	rfidModule.encode(dataBuffer.get(), numOfReturnedElem);
}

uint16_t RFIDManager::getBufferLength()
{
	return uartBuffer.getBufferLength();
}

uint8_t* RFIDManager::getDataBuffer()
{
	return uartBuffer.getDataBuffer();
}

uint8_t RFIDManager::getDataInArray(uint8_t *dataBuffer)
{
	if(rfidModule.isUpdated())
	{
		return rfidModule.getDataInArray(dataBuffer);
	}
	else
	{
		return 0;
	}
}

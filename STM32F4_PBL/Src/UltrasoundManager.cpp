/*
 * UltrasoundManager.cpp
 *
 *  Created on: Apr 26, 2020
 *      Author: piotrek
 */

#include "UltrasoundManager.h"

UltrasoundManager::UltrasoundManager(DMA_HandleTypeDef* hdmaRxUart, UART_HandleTypeDef* huart):
										uartBuffer(hdmaRxUart, ULTRASOUND_UART_BUFFER_LENGTH),
										huart(huart)
{
	// TODO Auto-generated constructor stub

}

UltrasoundManager::~UltrasoundManager() {
	// TODO Auto-generated destructor stub
}

void UltrasoundManager::processRxData()
{
	std::unique_ptr<uint8_t[]> dataBuffer = std::make_unique<uint8_t[]>(uartBuffer.getBufferLength());
	uint16_t numOfReturnedElem;

	uartBuffer.checkRxBuffer();
	numOfReturnedElem = uartBuffer.getData(dataBuffer.get());

	ultraSModule.encode(dataBuffer.get(), numOfReturnedElem);
}

void UltrasoundManager::fetchDistanceData()
{
	uint8_t dataToTransmit[4] = {0x22, 0x00, 0x00, 0x22};
	HAL_UART_Transmit(huart, dataToTransmit, sizeof(dataToTransmit), 1);
}

uint16_t UltrasoundManager::getBufferLength()
{
	return uartBuffer.getBufferLength();
}

uint8_t* UltrasoundManager::getDataBuffer()
{
	return uartBuffer.getDataBuffer();
}

uint8_t UltrasoundManager::getDataInArray(uint8_t *dataBuffer)
{
	if(ultraSModule.isUpdated())
	{
		return ultraSModule.getDataInArray(dataBuffer);
	}
	else
	{
		return 0;
	}
}

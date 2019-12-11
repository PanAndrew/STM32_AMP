/*
 * MiniLidarManager.cpp
 *
 *  Created on: Dec 16, 2019
 *      Author: piotrek
 */

#include "MiniLidarManager.h"

MiniLidarManager::MiniLidarManager(DMA_HandleTypeDef* hdmaRxUart): uartBuffer(hdmaRxUart, MINI_LIDAR_UART_BUFFER_LENGTH)
{
}

MiniLidarManager::~MiniLidarManager() {
	// TODO Auto-generated destructor stub
}

void MiniLidarManager::processRxData()
{
	std::unique_ptr<uint8_t[]> dataBuffer = std::make_unique<uint8_t[]>(uartBuffer.getBufferLength());
	uint16_t numOfReturnedElem;

	uartBuffer.checkRxBuffer();
	numOfReturnedElem = uartBuffer.getData(dataBuffer.get());

	miniLidar.encode(dataBuffer.get(), numOfReturnedElem);
}

uint16_t MiniLidarManager::getBufferLength()
{
	return uartBuffer.getBufferLength();
}

uint8_t* MiniLidarManager::getDataBuffer()
{
	return uartBuffer.getDataBuffer();
}

uint8_t MiniLidarManager::getDataInArray(uint8_t* dataBuffer)
{
	if(miniLidar.isUpdated())
	{
		testDiff = HAL_GetTick() - testLastTimestamp;
		testLastTimestamp = testLastTimestamp + testDiff;

		return miniLidar.getDataInArray(dataBuffer);
	}
	else
	{
		return 0;
	}
}

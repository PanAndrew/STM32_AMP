/*
 * UARTBuffer.cpp
 *
 *  Created on: Dec 9, 2019
 *      Author: piotrek
 */

#include "UARTBuffer.h"

UARTBuffer::UARTBuffer(DMA_HandleTypeDef* hdmaRxUart, uint16_t bufferSize): rxBuffer(std::make_unique<uint8_t[]>(bufferSize))
{
	this->hdmaRxUart = hdmaRxUart;
	this->bufferSize = bufferSize;
}

UARTBuffer::~UARTBuffer() {
	// TODO Auto-generated destructor stub
}

void UARTBuffer::checkRxBuffer()
{
	headPosition = bufferSize - __HAL_DMA_GET_COUNTER(hdmaRxUart);
}

uint16_t UARTBuffer::getBufferLength()
{
	return bufferSize;
}

uint16_t UARTBuffer::getSize()
{
	uint16_t size;

	if (headPosition >= tailPosition)
	{
		size = headPosition - tailPosition;
	}
	else
	{
		size = bufferSize + headPosition - tailPosition;
	}

	return size;
}

uint16_t UARTBuffer::getData(uint8_t* dataBuffer)
{
	uint16_t dataSize = getSize();

	if(dataSize)
	{
		if(headPosition > tailPosition)
		{
			std::copy_n(&rxBuffer[tailPosition], headPosition - tailPosition, dataBuffer);
		}
		else
		{
			std::copy_n(&rxBuffer[tailPosition], bufferSize - tailPosition, dataBuffer);

			if(headPosition > 0)
			{
				std::copy_n(rxBuffer.get(), headPosition, dataBuffer);
			}
		}

		tailPosition = headPosition;
	}

	if(tailPosition == bufferSize)
	{
		tailPosition = 0;
	}

	return dataSize;
}

uint8_t* UARTBuffer::getDataBuffer()
{
	return rxBuffer.get();
}

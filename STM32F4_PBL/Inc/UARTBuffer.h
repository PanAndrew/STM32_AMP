/*
 * UARTBuffer.h
 *
 *  Created on: Dec 9, 2019
 *      Author: piotrek
 */

#ifndef UARTBUFFER_H_
#define UARTBUFFER_H_

#include "stdint.h"
#include "main.h"
#include "string.h"
#include <algorithm>

#define UART_BUFFER_LENGTH 255

class UARTBuffer {

	DMA_HandleTypeDef* hdmaRxUart;

	uint16_t headPosition = 0;
	uint16_t tailPosition = 0;

	uint8_t rxBuffer[UART_BUFFER_LENGTH];
	uint16_t bufferSize;

public:
	UARTBuffer(DMA_HandleTypeDef* hdmaRxUart);
	virtual ~UARTBuffer();

	void checkRxBuffer();
	uint16_t getBufferLength();
	uint16_t getSize();
	uint16_t getData(uint8_t* dataBuffer);
	uint8_t* getDataBuffer();

};

#endif /* UARTBUFFER_H_ */

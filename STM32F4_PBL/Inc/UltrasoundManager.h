/*
 * UltrasoundManager.h
 *
 *  Created on: Apr 26, 2020
 *      Author: piotrek
 */

#ifndef ULTRASOUNDMANAGER_H_
#define ULTRASOUNDMANAGER_H_

#include "main.h"
#include "UARTBuffer.h"
#include "UltrasoundModule.h"
#include <memory>

#define ULTRASOUND_UART_BUFFER_LENGTH 8

class UltrasoundManager {

	UARTBuffer uartBuffer;
	UltrasoundModule ultraSModule;

	UART_HandleTypeDef* huart;

public:
	UltrasoundManager(DMA_HandleTypeDef* hdmaRxUart, UART_HandleTypeDef* huart);
	virtual ~UltrasoundManager();

	void processRxData();
	void fetchDistanceData();
	uint16_t getBufferLength();
	uint8_t* getDataBuffer();
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* ULTRASOUNDMANAGER_H_ */

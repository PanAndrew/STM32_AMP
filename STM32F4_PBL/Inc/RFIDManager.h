/*
 * RFIDManager.h
 *
 *  Created on: Mar 22, 2020
 *      Author: piotrek
 */

#ifndef RFIDMANAGER_H_
#define RFIDMANAGER_H_

#include "main.h"
#include "UARTBuffer.h"
#include "RFIDModule.h"
#include <memory>

#define MINI_RFID_UART_BUFFER_LENGTH 32

class RFIDManager {

	UARTBuffer uartBuffer;
	RFIDModule rfidModule;

public:
	RFIDManager(DMA_HandleTypeDef* hdmaRxUart);
	virtual ~RFIDManager();

	void processRxData();
	uint16_t getBufferLength();
	uint8_t* getDataBuffer();
	uint8_t getDataInArray(uint8_t *dataBuffer);

};

#endif /* RFIDMANAGER_H_ */

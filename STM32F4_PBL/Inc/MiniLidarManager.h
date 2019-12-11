/*
 * MiniLidarManager.h
 *
 *  Created on: Dec 16, 2019
 *      Author: piotrek
 */

#ifndef MINILIDARMANAGER_H_
#define MINILIDARMANAGER_H_

#include "main.h"
#include "UARTBuffer.h"
#include "MiniLidar.h"
#include <memory>

#define MINI_LIDAR_UART_BUFFER_LENGTH 18

class MiniLidarManager {

	UARTBuffer uartBuffer;
	MiniLidar miniLidar;


	uint32_t testDiff = 0;
	uint32_t testLastTimestamp = 0;

public:
	MiniLidarManager(DMA_HandleTypeDef* hdmaRxUart);
	virtual ~MiniLidarManager();

	void processRxData();
	uint16_t getBufferLength();
	uint8_t* getDataBuffer();
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* MINILIDARMANAGER_H_ */

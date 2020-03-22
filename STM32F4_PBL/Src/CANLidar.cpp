/*
 * CANLidar.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: piotrek
 */

#include "CANLidar.h"

CANLidar::CANLidar(CAN_HandleTypeDef *handleCan) : adasFramesBuffer(ADAS_FRAMES_BUFFER_SIZE)
{
	this->hcan = handleCan;
}

CANLidar::~CANLidar() {
	// TODO Auto-generated destructor stub
}

void CANLidar::configureCAN()
{
	CAN_FilterTypeDef filter;

	filter.FilterIdHigh = CAN_DISTANCE_FRAME_ID << 5;
	filter.FilterIdLow = 0x000;
	filter.FilterMaskIdHigh = 0xFFC << 5;
	filter.FilterMaskIdLow = 0x000;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterBank = 0;
	filter.FilterActivation = CAN_FILTER_ENABLE;

	HAL_CAN_ConfigFilter(hcan, &filter);
}

void CANLidar::processRXData(uint8_t rxFifo)
{
	uint8_t procDataArray[8];

	if(HAL_CAN_GetRxMessage(hcan, rxFifo, &rxHeader, procDataArray) == HAL_OK)
	{
		switch(rxHeader.StdId)
		{
		case 0x1E5:
			std::copy_n(procDataArray, rxHeader.DLC, rawDistanceData);
			updatedDistance = true;
			break;

		case 0x1E6:
			std::copy_n(procDataArray, rxHeader.DLC, rawSpeedData);
			updatedSpeed = true;
			break;

		case 0x1E7:
			std::copy_n(procDataArray, rxHeader.DLC, rawReflectionData);
			updatedReflection = true;
			break;

		default:
			break;
		}
	}
}

void CANLidar::scheduleADASFrames()
{
	if(counter % 1)
	{
		adasFrameState.f119();
		adasFramesBuffer.put(adasFrameState);
		adasFrameState.f122();
		adasFramesBuffer.put(adasFrameState);
		adasFrameState.f124();
		adasFramesBuffer.put(adasFrameState);
		adasFrameState.f1AF();
		adasFramesBuffer.put(adasFrameState);
		adasFrameState.f1D0();
		adasFramesBuffer.put(adasFrameState);
	}

	if (counter % 2)
	{
		adasFrameState.f1E8();
		adasFramesBuffer.put(adasFrameState);
		adasFrameState.f1EF();
		adasFramesBuffer.put(adasFrameState);
	}

	if (counter % 5)
	{
		adasFrameState.f2A2();
		adasFramesBuffer.put(adasFrameState);
	}

	if (counter % 10)
	{
		adasFrameState.f318();
		adasFramesBuffer.put(adasFrameState);
		adasFrameState.f32B();
		adasFramesBuffer.put(adasFrameState);
		adasFrameState.f380();
		adasFramesBuffer.put(adasFrameState);
		adasFrameState.f3A1();
		adasFramesBuffer.put(adasFrameState);
		adasFrameState.f3D1();
		adasFramesBuffer.put(adasFrameState);
	}

	if (counter % 20)
	{
		adasFrameState.f463();
		adasFramesBuffer.put(adasFrameState);

		counter = 0;
	}

	counter++;

	sendCANFrames();
}

void CANLidar::sendCANFrames()
{
	ADASManager tempADASFrame;
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan))
	{
		if(!adasFramesBuffer.empty())
		{
			tempADASFrame = adasFramesBuffer.get();
			HAL_CAN_AddTxMessage(hcan, &tempADASFrame.txHeader, tempADASFrame.adasFrameArray, &mailBox);
		}
		else
		{
			break;
		}
	}
}

uint8_t CANLidar::getDistanceDataInArray(uint8_t *dataBuffer)
{
	if(!updatedDistance)
	{
		return 0;
	}
	updatedDistance = false;

	std::copy_n(rawDistanceData, CAN_DATA_FRAME_SIZE, dataBuffer);

	return CAN_DATA_FRAME_SIZE;
}

uint8_t CANLidar::getSpeedDataInArray(uint8_t *dataBuffer)
{
	if(!updatedSpeed)
	{
		return 0;
	}
	updatedSpeed = false;

	std::copy_n(rawSpeedData, CAN_DATA_FRAME_SIZE, dataBuffer);

	return CAN_DATA_FRAME_SIZE;
}

uint8_t CANLidar::getReflectionDataInArray(uint8_t *dataBuffer)
{
	if(!updatedReflection)
	{
		return 0;
	}
	updatedReflection = false;

	std::copy_n(rawReflectionData, CAN_DATA_FRAME_SIZE, dataBuffer);

	return CAN_DATA_FRAME_SIZE;
}

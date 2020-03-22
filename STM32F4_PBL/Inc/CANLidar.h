/*
 * CANLidar.h
 *
 *  Created on: Mar 20, 2020
 *      Author: piotrek
 */

#ifndef CANLIDAR_H_
#define CANLIDAR_H_

#include "DataBuffer.h"
#include "ADASManager.h"
#include <algorithm>
#include "main.h"

#define CAN_DISTANCE_FRAME_ID 0x1E5
#define CAN_SPEED_FRAME_ID 0x1E6
#define CAN_REFLECTION_FRAME_ID 0x1E7

#define CAN_DATA_FRAME_SIZE 6
#define ADAS_FRAMES_BUFFER_SIZE 50

class CANLidar {
	CAN_HandleTypeDef *hcan;
	CAN_RxHeaderTypeDef rxHeader;
	uint32_t mailBox;

	DataBuffer<ADASManager> adasFramesBuffer;
	uint8_t counter = 0;
	ADASManager adasFrameState;

	uint8_t rawDistanceData[8];
	uint8_t rawSpeedData[8];
	uint8_t rawReflectionData[8];

	bool updatedDistance = false;
	bool updatedSpeed = false;
	bool updatedReflection = false;

public:
	CANLidar(CAN_HandleTypeDef *handleCan);
	virtual ~CANLidar();

	void configureCAN();
	void processRXData(uint8_t rxFifo);
	void scheduleADASFrames();
	void sendCANFrames();

	uint8_t getDistanceDataInArray(uint8_t* dataBuffer);
	uint8_t getSpeedDataInArray(uint8_t* dataBuffer);
	uint8_t getReflectionDataInArray(uint8_t* dataBuffer);
};

#endif /* CANLIDAR_H_ */

/*
 * EncoderSystem.cpp
 *
 *  Created on: Nov 27, 2019
 *      Author: piotrek
 */

#include "EncoderSystem.h"

EncoderSystem::EncoderSystem(TIM_HandleTypeDef* htimL, TIM_HandleTypeDef* htimR)
{
	encoders[REAR_LEFT].initialize(htimL);
	encoders[REAR_RIGHT].initialize(htimR);
}

EncoderSystem::~EncoderSystem() {
	// TODO Auto-generated destructor stub
}

void EncoderSystem::encoderService()
{
	for(uint8_t i = 0; i < NUM_OF_ENCODERS; i++)
	{
		encoders[i].encoderIteration();
	}
}

uint8_t EncoderSystem::getDataInArray(uint8_t* dataBuffer)
{
	uint8_t dataToReturnMain[ENCODER_SYS_OBJECTDATAVOLUME];
	uint8_t dataToReturnLeft[ENCODER_OBJECTDATAVOLUME];
	uint8_t dataToReturnRight[ENCODER_OBJECTDATAVOLUME];

	encoders[REAR_LEFT].getDataInArray(dataToReturnLeft);
	encoders[REAR_RIGHT].getDataInArray(dataToReturnRight);

	dataToReturnMain[0] = dataToReturnLeft[ENCODER_DIRECTION] << 4	| dataToReturnRight[ENCODER_DIRECTION];
	dataToReturnMain[1] = dataToReturnLeft[ENCODER_SPEED_3_CMD];
	dataToReturnMain[2] = dataToReturnLeft[ENCODER_SPEED_2_CMD];
	dataToReturnMain[3] = dataToReturnLeft[ENCODER_SPEED_1_CMD];
	dataToReturnMain[4] = dataToReturnLeft[ENCODER_SPEED_0_CMD];

	dataToReturnMain[5] = dataToReturnLeft[ENCODER_DISTANCE_3_CMD];
	dataToReturnMain[6] = dataToReturnLeft[ENCODER_DISTANCE_2_CMD];
	dataToReturnMain[7] = dataToReturnLeft[ENCODER_DISTANCE_1_CMD];
	dataToReturnMain[8] = dataToReturnLeft[ENCODER_DISTANCE_0_CMD];

	dataToReturnMain[9] = dataToReturnRight[ENCODER_SPEED_3_CMD];
	dataToReturnMain[10] = dataToReturnRight[ENCODER_SPEED_2_CMD];
	dataToReturnMain[11] = dataToReturnRight[ENCODER_SPEED_1_CMD];
	dataToReturnMain[12] = dataToReturnRight[ENCODER_SPEED_0_CMD];

	dataToReturnMain[13] = dataToReturnRight[ENCODER_DISTANCE_3_CMD];
	dataToReturnMain[14] = dataToReturnRight[ENCODER_DISTANCE_2_CMD];
	dataToReturnMain[15] = dataToReturnRight[ENCODER_DISTANCE_1_CMD];
	dataToReturnMain[16] = dataToReturnRight[ENCODER_DISTANCE_0_CMD];

	std::copy_n(dataToReturnMain, ENCODER_SYS_OBJECTDATAVOLUME, dataBuffer);

	return ENCODER_SYS_OBJECTDATAVOLUME;
}


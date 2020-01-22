/*
 * EncoderSystem.h
 *
 *  Created on: Nov 27, 2019
 *      Author: piotrek
 */

#ifndef ENCODERSYSTEM_H_
#define ENCODERSYSTEM_H_

#include "Encoder.h"

#define REAR_LEFT 0
#define REAR_RIGHT 1
#define NUM_OF_ENCODERS 2

#define ENCODER_DIRECTION 0
#define ENCODER_SPEED_3_CMD 1
#define ENCODER_SPEED_2_CMD 2
#define ENCODER_SPEED_1_CMD 3
#define ENCODER_SPEED_0_CMD 4

#define ENCODER_DISTANCE_3_CMD 5
#define ENCODER_DISTANCE_2_CMD 6
#define ENCODER_DISTANCE_1_CMD 7
#define ENCODER_DISTANCE_0_CMD 8

#define ENCODER_SYS_OBJECTDATAVOLUME 17

class EncoderSystem
{
	Encoder encoders[NUM_OF_ENCODERS];

public:
	EncoderSystem(TIM_HandleTypeDef* htimL, TIM_HandleTypeDef* htimR);
	virtual ~EncoderSystem();

	void encoderService();

	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* ENCODERSYSTEM_H_ */

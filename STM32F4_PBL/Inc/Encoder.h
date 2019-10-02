/*
 * Encoder.h
 *
 *  Created on: May 14, 2019
 *      Author: piotrek
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stdint.h"
#include <algorithm>

#define LEFT_ENCODER 0
#define RIGHT_ENCODER 1

#define ENCODER_OBJECTDATAVOLUME 4

class Encoder {

	volatile uint16_t encoderPulse[2] = { 0, 0 };
	volatile uint16_t encoderPPSec[2] = { 0, 0 };

public:
	Encoder();
	virtual ~Encoder();

	void incrementEncoderPulse(uint8_t encoderNum);
	void countPulsesPerSecond(void);
	uint16_t getPulsePerSecond(uint8_t &encoderNum);
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* ENCODER_H_ */

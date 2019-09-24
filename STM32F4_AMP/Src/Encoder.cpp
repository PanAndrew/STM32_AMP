/*
 * Encoder.cpp
 *
 *  Created on: May 14, 2019
 *      Author: piotrek
 */

#include <Encoder.h>

Encoder::Encoder() {
	// TODO Auto-generated constructor stub

}

Encoder::~Encoder() {
	// TODO Auto-generated destructor stub
}

void Encoder::incrementEncoderPulse(uint8_t encoderNum)
{
	encoderPulse[encoderNum]++;
}

void Encoder::countPulsesPerSecond(void)
{
	encoderPPSec[LEFT_ENCODER] = encoderPulse[LEFT_ENCODER];
	encoderPPSec[RIGHT_ENCODER] = encoderPulse[RIGHT_ENCODER];
	encoderPulse[LEFT_ENCODER] = 0;
	encoderPulse[RIGHT_ENCODER] = 0;
}

uint16_t Encoder::getPulsePerSecond(uint8_t &encoderNum)
{
	return encoderPPSec[encoderNum];
}

uint8_t Encoder::getDataInArray(uint8_t* dataBuffer)
{
	uint8_t dataToReturn[ENCODER_OBJECTDATAVOLUME];

	dataToReturn[0] = encoderPPSec[LEFT_ENCODER] >> 8;
	dataToReturn[1] = encoderPPSec[LEFT_ENCODER] & 0xFF;
	dataToReturn[2] = encoderPPSec[RIGHT_ENCODER] >> 8;
	dataToReturn[3] = encoderPPSec[RIGHT_ENCODER] & 0xFF;

	std::copy_n(dataToReturn, ENCODER_OBJECTDATAVOLUME, dataBuffer);

	return ENCODER_OBJECTDATAVOLUME;
}

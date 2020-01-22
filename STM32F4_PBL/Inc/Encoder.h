/*
 * Encoder.h
 *
 *  Created on: May 14, 2019
 *      Author: piotrek
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stdint.h"
#include "main.h"
#include <algorithm>
#include "DataBuffer.h"
#include "unions.h"

#define LEFT_ENCODER 0
#define RIGHT_ENCODER 1

#define DEFAULT_NUM_OF_CHECKS 20

#define INNER_WHEEL_RATIO 4.77
#define EXTERN_WHEEL_RATIO 7.2

#define HALF_OF_TIM_ARR 32767

#define ENCODER_OBJECTDATAVOLUME 9

class Encoder {

	enum status
	{
		idle_stat,
		forwardRun,
		backRun
	};

	TIM_HandleTypeDef* htim;

	uint16_t presentTimRegisterValue = 0;
	uint16_t lastTimRegisterValue = 0;
	int16_t diff = 0;
	uint32_t presentTimeStamp = 0;
	uint32_t lastTimeStamp = 0;
	uint32_t elapsedTime = 0;

	DataBuffer<floatUnion> speedBuffer;
	floatUnion tempSpeed;

	floatUnion speed;
	floatUnion distance;
	uint8_t numberOfGoOnChecks = DEFAULT_NUM_OF_CHECKS;
	status encoderState = idle_stat;

	bool checkIfMoveMade();
	bool checkIfOverflow();
	void calcSpeed();

public:
	Encoder();
	virtual ~Encoder();

	void initialize(TIM_HandleTypeDef* htim);
	void encoderIteration();

	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* ENCODER_H_ */

/*
 * DrivingCommand.h
 *
 *  Created on: Nov 4, 2019
 *      Author: piotrek
 */

#ifndef DRIVINGCOMMAND_H_
#define DRIVINGCOMMAND_H_

#include "stdint.h"

class DrivingCommand {

	uint8_t firstDirection;
	uint8_t secondDirection;
	uint16_t firstPWM;
	uint16_t secondPWM;
	uint16_t timeToDrive;

public:
	DrivingCommand();
	DrivingCommand(uint8_t firstDir, uint16_t firstPWMValue,
			uint8_t secondDir, uint16_t secondPWMValue, uint16_t timeToDriveValue);
	virtual ~DrivingCommand();

	uint16_t* getSecondPWM();
	uint8_t* getSecondDirection();
	uint8_t* getFirstDirection();
	uint16_t* getFirstPWM();
	uint16_t getTimeToDrive();
};

#endif /* DRIVINGCOMMAND_H_ */

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

	uint8_t servoDirection;
	uint8_t motorDirection;
	uint16_t dcMotorPWM;
	uint16_t servoPWM;
	uint16_t timeToDrive;

public:
	DrivingCommand(uint8_t servoDirection, uint16_t servoPWM, uint8_t motorDirection, uint16_t dcMotorPWM, uint16_t timeToDrive);
	virtual ~DrivingCommand();

	uint16_t* getMotorDesiredPWM();
	uint8_t* getMotorDirection();
	uint8_t* getServoDirection();
	uint16_t* getServoDesiredPWM();
	uint16_t getTimeToDrive();
};

#endif /* DRIVINGCOMMAND_H_ */

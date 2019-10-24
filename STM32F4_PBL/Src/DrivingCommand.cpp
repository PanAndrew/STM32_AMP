/*
 * DrivingCommand.cpp
 *
 *  Created on: Nov 4, 2019
 *      Author: piotrek
 */

#include "DrivingCommand.h"

DrivingCommand::DrivingCommand(uint8_t servoDir, uint16_t servoPWMValue,
		uint8_t motorDir, uint16_t dcMotorPWMValue, uint16_t timeToDriveValue) {

	servoDirection = servoDir;
	servoPWM = servoPWMValue;
	motorDirection = motorDir;
	dcMotorPWM = dcMotorPWMValue;
	timeToDrive = timeToDriveValue;
}

DrivingCommand::~DrivingCommand() {
	// TODO Auto-generated destructor stub
}


uint16_t* DrivingCommand::getMotorDesiredPWM()
{
	return &dcMotorPWM;
}

uint8_t* DrivingCommand::getMotorDirection()
{
	return &motorDirection;
}

uint8_t* DrivingCommand::getServoDirection()
{
	return &servoDirection;
}

uint16_t* DrivingCommand::getServoDesiredPWM()
{
	return &servoPWM;
}

uint16_t DrivingCommand::getTimeToDrive()
{
	return timeToDrive;
}

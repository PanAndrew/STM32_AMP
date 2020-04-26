/*
 * DrivingCommand.cpp
 *
 *  Created on: Nov 4, 2019
 *      Author: piotrek
 */

#include "DrivingCommand.h"

DrivingCommand::DrivingCommand()
{
	firstDirection = 0;
	firstPWM = 0;
	secondDirection = 0;
	secondPWM = 0;
	timeToDrive = 0;
}

DrivingCommand::DrivingCommand(uint8_t firstDir, uint16_t firstPWMValue,
		uint8_t secondDir, uint16_t secondPWMValue, uint16_t timeToDriveValue)
{

	firstDirection = firstDir;
	firstPWM = firstPWMValue;
	secondDirection = secondDir;
	secondPWM = secondPWMValue;
	timeToDrive = timeToDriveValue;
}

DrivingCommand::~DrivingCommand() {
	// TODO Auto-generated destructor stub
}


uint16_t* DrivingCommand::getSecondPWM()
{
	return &secondPWM;
}

uint8_t* DrivingCommand::getSecondDirection()
{
	return &secondDirection;
}

uint8_t* DrivingCommand::getFirstDirection()
{
	return &firstDirection;
}

uint16_t* DrivingCommand::getFirstPWM()
{
	return &firstPWM;
}

uint16_t DrivingCommand::getTimeToDrive()
{
	return timeToDrive;
}

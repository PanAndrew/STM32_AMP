/*
 * DrivingSystem.cpp
 *
 *  Created on: Oct 1, 2019
 *      Author: piotrek
 */

#include "DrivingSystem.h"

DrivingSystem::DrivingSystem(TIM_HandleTypeDef *htimDC, uint8_t timChannelDC,
		TIM_HandleTypeDef *htimServo, uint8_t timChannelServo)
{
	dcMotorDriver.initialize(htimDC, timChannelDC);
	servo.initialize(htimServo, timChannelServo);
}

DrivingSystem::~DrivingSystem() {
	// TODO Auto-generated destructor stub
}

void DrivingSystem::initialize()
{
	dcMotorDriver.idle();
	servo.idle();
}

void DrivingSystem::drivingService()
{
	if(checkTimeToDrive())
	{
		dcMotorDriver.idle();
		servo.idle();
	}
}

bool DrivingSystem::checkTimeToDrive()
{
	if(HAL_GetTick() - lastDrivingCmdTimeStamp > timeToDrive)
	{
		return true;
	} else {
		return false;
	}
}

void DrivingSystem::calculateRemainedTTD()
{
	if(checkTimeToDrive())
	{
		remainedTTD = 0;
	} else {
		remainedTTD = timeToDrive - (HAL_GetTick() - lastDrivingCmdTimeStamp);
	}
}

void DrivingSystem::configureDesiredPWM(uint8_t *dataBuffer)
{
	uint8_t direction;
	uint16_t desiredPWM;

	for(uint8_t i = 0; i < NUMBER_OF_DC_MOTORS; i++)
	{
		direction = dataBuffer[DATASET_LENGTH * i + DIRECTION_COMMAND] & 0xF;
		desiredPWM = dataBuffer[DATASET_LENGTH * i + MOTOR_PWM_COMMAND_H] << 8 |
				dataBuffer[DATASET_LENGTH * i + MOTOR_PWM_COMMAND_L];
		timeToDrive = dataBuffer[DATASET_LENGTH * i + TIME_TO_DRIVE_H] << 8 |
				dataBuffer[DATASET_LENGTH * i + TIME_TO_DRIVE_L];
		dcMotorDriver.configureDesiredPWM(&direction, &desiredPWM);
	}

	for(uint8_t i = 0; i < NUMBER_OF_SERVOS; i++)
	{
		direction = dataBuffer[DATASET_LENGTH * i + DIRECTION_COMMAND] >> 4;
		desiredPWM = dataBuffer[DATASET_LENGTH * i + SERVO_PWM_COMMAND_H] << 8 |
				dataBuffer[DATASET_LENGTH * i + SERVO_PWM_COMMAND_L];
		servo.configureDesiredPWM(&direction, &desiredPWM);
	}

	lastDrivingCmdTimeStamp = HAL_GetTick();
}

uint8_t DrivingSystem::getDataInArray(uint8_t* dataBuffer)
{
	uint8_t dataToReturnMain[DATASET_LENGTH];
	uint8_t dataToReturnServo[SERVO_OBJECTDATAVOLUME];
	uint8_t dataToReturnDC[MOTOR_OBJECTDATAVOLUME];

	servo.getDataInArray(dataToReturnServo);
	dcMotorDriver.getDataInArray(dataToReturnDC);

	dataToReturnMain[0] = dataToReturnServo[DIRECTION_COMMAND] << 4 | dataToReturnDC[DIRECTION_COMMAND];
	dataToReturnMain[1] = dataToReturnServo[SERVO_PWM_COMMAND_H];
	dataToReturnMain[2] = dataToReturnServo[SERVO_PWM_COMMAND_L];
	dataToReturnMain[3] = dataToReturnDC[MOTOR_PWM_COMMAND_H];
	dataToReturnMain[4] = dataToReturnDC[MOTOR_PWM_COMMAND_L];

	calculateRemainedTTD();
	dataToReturnMain[5] = remainedTTD >> 8;
	dataToReturnMain[6] = remainedTTD & 0xFF;

	std::copy_n(dataToReturnMain, DATASET_LENGTH, dataBuffer);

	return DATASET_LENGTH;
}

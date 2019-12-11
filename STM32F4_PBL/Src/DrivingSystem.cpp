/*
 * DrivingSystem.cpp
 *
 *  Created on: Oct 1, 2019
 *      Author: piotrek
 */

#include "DrivingSystem.h"

DrivingSystem::DrivingSystem(TIM_HandleTypeDef *htimDC, uint8_t timChannelDC,
		TIM_HandleTypeDef *htimServo, uint8_t timChannelServo) : commandsQueueRB(MAX_QUEUE_SIZE)
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
		if(!commandsQueueRB.empty())
		{
			loadFrontCommand();
		}
		else
		{
			dcMotorDriver.stop();
			servo.stop();
		}
	}

	dcMotorDriver.drivingIteration();
	if(dcMotorDriver.isShiftingQueueEmpty())
	{
		servo.steeringIteration();
	}
	else
	{
		lastDrivingCmdTimeStamp = HAL_GetTick();
	}
}

void DrivingSystem::loadFrontCommand()
{
	DrivingCommand driveCmd = commandsQueueRB.get();

	dcMotorDriver.configureDesiredPWM(driveCmd.getMotorDirection(), driveCmd.getMotorDesiredPWM());
	servo.configureDesiredPWM(driveCmd.getServoDirection(), driveCmd.getServoDesiredPWM());
	timeToDrive = driveCmd.getTimeToDrive();

	lastDrivingCmdTimeStamp = HAL_GetTick();
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
	uint8_t servoDirection;
	uint8_t motorDirection;
	uint16_t servoDesiredPWM;
	uint16_t motorDesiredPWM;
	uint16_t timeToDriveValue;
	uint8_t queueCommandFlag;

	servoDirection = dataBuffer[DIRECTION_COMMAND] >> 4;
	servoDesiredPWM = dataBuffer[SERVO_PWM_COMMAND_H] << 8 | dataBuffer[SERVO_PWM_COMMAND_L];

	motorDirection = dataBuffer[DIRECTION_COMMAND] & 0xF;
	motorDesiredPWM = dataBuffer[MOTOR_PWM_COMMAND_H] << 8 | dataBuffer[MOTOR_PWM_COMMAND_L];

	timeToDriveValue = dataBuffer[TIME_TO_DRIVE_H] << 8 | dataBuffer[TIME_TO_DRIVE_L];
	queueCommandFlag = dataBuffer[QUEUE_COMMAND_FLAG];

	if(queueCommandFlag)
	{
		if(commandsQueueRB.size() < MAX_QUEUE_SIZE)
		{
			DrivingCommand driveCmd = DrivingCommand(servoDirection, servoDesiredPWM,
					motorDirection, motorDesiredPWM, timeToDriveValue);

			commandsQueueRB.put(driveCmd);
		}
	}
	else
	{
		commandsQueueRB.reset();

		dcMotorDriver.configureDesiredPWM(&motorDirection, &motorDesiredPWM);
		servo.configureDesiredPWM(&servoDirection, &servoDesiredPWM);
		timeToDrive = timeToDriveValue;

		lastDrivingCmdTimeStamp = HAL_GetTick();
	}
}

uint8_t DrivingSystem::getDataInArray(uint8_t* dataBuffer)
{
	uint8_t dataToReturnMain[DATASET_LENGTH];
	uint8_t dataToReturnServo[SERVO_OBJECTDATAVOLUME];
	uint8_t dataToReturnDC[MOTOR_OBJECTDATAVOLUME];

	servo.getDataInArray(dataToReturnServo);
	dcMotorDriver.getDataInArray(dataToReturnDC);

	dataToReturnMain[0] = dataToReturnServo[DIRECTION_COMMAND] << 4 | dataToReturnDC[DIRECTION_COMMAND];
	dataToReturnMain[1] = dataToReturnServo[PWM_COMMAND_H];
	dataToReturnMain[2] = dataToReturnServo[PWM_COMMAND_L];
	dataToReturnMain[3] = dataToReturnDC[PWM_COMMAND_H];
	dataToReturnMain[4] = dataToReturnDC[PWM_COMMAND_L];

	calculateRemainedTTD();
	dataToReturnMain[5] = remainedTTD >> 8;
	dataToReturnMain[6] = remainedTTD & 0xFF;
	dataToReturnMain[7] = commandsQueueRB.size();

	std::copy_n(dataToReturnMain, DATASET_LENGTH, dataBuffer);

	return DATASET_LENGTH;
}

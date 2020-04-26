/*
 * DrivingSystem.cpp
 *
 *  Created on: Oct 1, 2019
 *      Author: piotrek
 */

#include "DrivingSystem.h"

DrivingSystem::DrivingSystem(TIM_HandleTypeDef *htimFirst, uint8_t timFirstChannel,
		TIM_HandleTypeDef *htimSecond, uint8_t timSecondChannel) : commandsQueueRB(MAX_QUEUE_SIZE)
{

#ifdef DIFF_MOTOR
	diffMotors.initialize(htimFirst, timFirstChannel, timSecondChannel);
#else
	dcMotorDriver.initialize(htimSecond, timSecondChannel);
	servo.initialize(htimFirst, timFirstChannel);
#endif
}

DrivingSystem::~DrivingSystem() {
	// TODO Auto-generated destructor stub
}

void DrivingSystem::initialize()
{
#ifdef DIFF_MOTOR
	diffMotors.stopPlatform();
#else
	dcMotorDriver.idle();
	servo.idle();
#endif
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
		#ifdef DIFF_MOTOR
			diffMotors.stopPlatform();
		#else
			dcMotorDriver.stop();
			servo.stop();
		#endif
		}
	}

#ifdef DIFF_MOTOR
	diffMotors.drivingIteration();
#else
	dcMotorDriver.drivingIteration();
	if(dcMotorDriver.isShiftingQueueEmpty())
	{
		servo.steeringIteration();
	}
	else
	{
		lastDrivingCmdTimeStamp = HAL_GetTick();
	}
#endif
}

void DrivingSystem::loadFrontCommand()
{
	DrivingCommand driveCmd = commandsQueueRB.get();

#ifdef DIFF_MOTOR
	diffMotors.configureDesiredPWM(driveCmd.getFirstDirection(), driveCmd.getSecondDirection(),
									driveCmd.getFirstPWM(), driveCmd.getSecondPWM());
#else
	dcMotorDriver.configureDesiredPWM(driveCmd.getSecondDirection(), driveCmd.getSecondPWM());
		servo.configureDesiredPWM(driveCmd.getFirstDirection(), driveCmd.getFirstPWM());
#endif

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
	uint8_t firstDirection;
	uint8_t secondDirection;
	uint16_t firstDesiredPWM;
	uint16_t secondDesiredPWM;
	uint16_t timeToDriveValue;
	uint8_t queueCommandFlag;

	firstDirection = dataBuffer[DIRECTION_COMMAND] >> 4;
	firstDesiredPWM = dataBuffer[SERVO_PWM_COMMAND_H] << 8 | dataBuffer[SERVO_PWM_COMMAND_L];

	secondDirection = dataBuffer[DIRECTION_COMMAND] & 0xF;
	secondDesiredPWM = dataBuffer[MOTOR_PWM_COMMAND_H] << 8 | dataBuffer[MOTOR_PWM_COMMAND_L];

	timeToDriveValue = dataBuffer[TIME_TO_DRIVE_H] << 8 | dataBuffer[TIME_TO_DRIVE_L];
	queueCommandFlag = dataBuffer[QUEUE_COMMAND_FLAG];

	if(queueCommandFlag)
	{
		if(commandsQueueRB.size() < MAX_QUEUE_SIZE)
		{
			DrivingCommand driveCmd = DrivingCommand(firstDirection, firstDesiredPWM,
					secondDirection, secondDesiredPWM, timeToDriveValue);

			commandsQueueRB.put(driveCmd);
		}
	}
	else
	{
		commandsQueueRB.reset();

#ifdef DIFF_MOTOR
	diffMotors.configureDesiredPWM(&firstDirection, &secondDirection,
									&firstDesiredPWM, &secondDesiredPWM);
#else
	dcMotorDriver.configureDesiredPWM(&secondDirection, &secondDesiredPWM);
	servo.configureDesiredPWM(&firstDirection, &firstDesiredPWM);
#endif

		timeToDrive = timeToDriveValue;
		lastDrivingCmdTimeStamp = HAL_GetTick();
	}
}

uint8_t DrivingSystem::getDataInArray(uint8_t* dataBuffer)
{
	uint8_t dataToReturnMain[DATASET_LENGTH];

#ifdef DIFF_MOTOR
//	uint8_t dataToReturnDiffMotors[DIFF_MOTOR_OBJECTDATAVOLUME];
	diffMotors.getDataInArray(dataToReturnMain);
//	std::copy_n(dataToReturnDiffMotors, DIFF_MOTOR_OBJECTDATAVOLUME, dataToReturnMain);
#else
	uint8_t dataToReturnServo[SERVO_OBJECTDATAVOLUME];
	uint8_t dataToReturnDC[MOTOR_OBJECTDATAVOLUME];

	servo.getDataInArray(dataToReturnServo);
	dcMotorDriver.getDataInArray(dataToReturnDC);

	dataToReturnMain[0] = dataToReturnServo[DIRECTION_COMMAND] << 4 | dataToReturnDC[DIRECTION_COMMAND];
	dataToReturnMain[1] = dataToReturnServo[PWM_COMMAND_H];
	dataToReturnMain[2] = dataToReturnServo[PWM_COMMAND_L];
	dataToReturnMain[3] = dataToReturnDC[PWM_COMMAND_H];
	dataToReturnMain[4] = dataToReturnDC[PWM_COMMAND_L];
#endif

	calculateRemainedTTD();
	dataToReturnMain[5] = remainedTTD >> 8;
	dataToReturnMain[6] = remainedTTD & 0xFF;
	dataToReturnMain[7] = commandsQueueRB.size();

	std::copy_n(dataToReturnMain, DATASET_LENGTH, dataBuffer);

	return DATASET_LENGTH;
}

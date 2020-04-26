/*
 * DrivingSystem.h
 *
 *  Created on: Oct 1, 2019
 *      Author: piotrek
 */

#ifndef DRIVINGSYSTEM_H_
#define DRIVINGSYSTEM_H_

#define DIFF_MOTOR

#ifdef DIFF_MOTOR
	#include "DifferentialDCMotor.h"
#else
	#include "DCMotorDriver.h"
	#include "Servo.h"
#endif

#include "DrivingCommand.h"
#include "DataBuffer.h"

#define DATASET_LENGTH 8

class DrivingSystem {

#ifdef DIFF_MOTOR
	DifferentialDCMotor diffMotors;
#else
	DCMotorDriver dcMotorDriver;
	Servo servo;
#endif

	uint16_t timeToDrive;
	uint32_t lastDrivingCmdTimeStamp;
	uint32_t remainedTTD;

	DataBuffer<DrivingCommand> commandsQueueRB;

	void calculateRemainedTTD();
	bool checkTimeToDrive();
	void loadFrontCommand();

public:
	DrivingSystem(TIM_HandleTypeDef *htimDC, uint8_t timChannelDC, TIM_HandleTypeDef *htimServo, uint8_t timChannelServo);
	virtual ~DrivingSystem();

	void initialize();

	void drivingService();
	void configureDesiredPWM(uint8_t* dataBuffer);
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* DRIVINGSYSTEM_H_ */

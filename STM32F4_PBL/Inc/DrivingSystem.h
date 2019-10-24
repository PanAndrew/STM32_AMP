/*
 * DrivingSystem.h
 *
 *  Created on: Oct 1, 2019
 *      Author: piotrek
 */

#ifndef DRIVINGSYSTEM_H_
#define DRIVINGSYSTEM_H_

#include "DCMotorDriver.h"
#include "Servo.h"

#define NUMBER_OF_DC_MOTORS 1
#define NUMBER_OF_SERVOS 1

#define DATASET_LENGTH 7

class DrivingSystem {

	DCMotorDriver dcMotorDriver;
	Servo servo;

	uint16_t timeToDrive;
	uint32_t lastDrivingCmdTimeStamp;
	uint32_t remainedTTD;

	void calculateRemainedTTD();
	bool checkTimeToDrive();

public:
	DrivingSystem(TIM_HandleTypeDef *htimDC, uint8_t timChannelDC, TIM_HandleTypeDef *htimServo, uint8_t timChannelServo);
	virtual ~DrivingSystem();

	void initialize();

	void drivingService();
	void configureDesiredPWM(uint8_t* dataBuffer);
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* DRIVINGSYSTEM_H_ */

/*
 * DCMotorDriver.h
 *
 *  Created on: Oct 1, 2019
 *      Author: piotrek
 */

#ifndef DCMOTOR_H_
#define DCMOTOR_H_

#include "stdint.h"
#include "main.h"
#include <algorithm>
#include "pwmDefines.h"
#include <list>

#define MAX_VOLTAGE 6
#define BATTERY_VOLTAGE 12

class DCMotorDriver {

	enum status
	{
		idle_stat,
		forwardRun,
		backRun,
		brakingFR_stat,
		brakingBR_stat,
		idling_stat
	};

	uint8_t idleStateValue = 0;
	uint16_t idlePWMValue = 0;

	uint16_t pwmValue;
	uint16_t pwmDesiredValue;
	status motorPresentStatus;
	status motorDesiredStatus;
	TIM_HandleTypeDef *htim;
	uint8_t timChannel;

	std::list<status> shiftingQueue;
	uint32_t shiftingTimestamp;

	void forward(uint16_t pwmValue);
	void back(uint16_t pwmValue);
	void makeManeuver(uint8_t direction, uint16_t pwmData);
	void setPWMOnTimer(uint16_t pwmValue);
	void braking();
	void idling();
	void confShiftingQueue();
	void addToShiftingQueue(status statusToQueue);
	void nextAction();
	bool checkShiftingTime(uint16_t shiftingTime);

public:
	DCMotorDriver();
	virtual ~DCMotorDriver();

	void initialize(TIM_HandleTypeDef *htim, uint8_t timChannel);

	void idle();
	void stop();
	void drivingIteration();
	bool isShiftingQueueEmpty();

	void configureDesiredPWM(uint8_t *direction, uint16_t *desiredPWM);
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* DCMotorDriver_H_ */

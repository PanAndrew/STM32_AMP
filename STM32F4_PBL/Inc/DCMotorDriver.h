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

#define MAX_VOLTAGE 6
#define BATTERY_VOLTAGE 12

class DCMotorDriver {

	enum status
	{
		idle_stat,
		forwardRun,
		backRun
	};

	uint16_t pwmValue;
	status motorStatus;
	TIM_HandleTypeDef *htim;
	uint8_t timChannel;


	void forward(uint16_t pwmValue);
	void back(uint16_t pwmValue);
	void makeManeuver(uint8_t direction, uint16_t pwmData);
	void setPWMOnTimer(uint16_t pwmValue);
	void braking();

public:
	DCMotorDriver();
	virtual ~DCMotorDriver();

	void initialize(TIM_HandleTypeDef *htim, uint8_t timChannel);

	void idle();

	void configureDesiredPWM(uint8_t *direction, uint16_t *desiredPWM);
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* DCMotorDriver_H_ */

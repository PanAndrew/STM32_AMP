/*
 * Servo.h
 *
 *  Created on: Oct 1, 2019
 *      Author: piotrek
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "stdint.h"
#include "main.h"
#include <algorithm>
#include "pwmDefines.h"

//#define UGV_4X4

class Servo {

	enum status
	{
		idle_stat,
		leftRun,
		rightRun
	};

	uint8_t idleStateValue = 0;
	uint16_t idlePWMValue = 0;

	uint16_t pwmValue;
	uint16_t pwmDesiredValue;
	status servoPresentStatus;
	status servoDesiredStatus;
	TIM_HandleTypeDef *htim;
	uint8_t timChannel;


	void left(uint16_t pwmValue);
	void right(uint16_t pwmValue);
	void makeManeuver(uint8_t direction, uint16_t pwmData);
	void setPWMOnTimer(uint16_t pwmValue);

public:
	Servo();
	virtual ~Servo();

	void initialize(TIM_HandleTypeDef *htim, uint8_t timChannel);

	void idle();
	void stop();
	void steeringIteration();

	void configureDesiredPWM(uint8_t *direction, uint16_t *desiredPWM);
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* SERVO_H_ */

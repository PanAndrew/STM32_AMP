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

class Servo {

	enum status
	{
		idle_stat,
		leftRun,
		rightRun
	};

	uint16_t pwmValue;
	status servoStatus;
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

	void configureDesiredPWM(uint8_t *direction, uint16_t *desiredPWM);
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* SERVO_H_ */

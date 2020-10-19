/*
 * DifferentialDCMotor.h
 *
 *  Created on: Apr 25, 2020
 *      Author: piotrek
 */

#ifndef DIFFERENTIALDCMOTOR_H_
#define DIFFERENTIALDCMOTOR_H_

#include "stdint.h"
#include "main.h"
#include <algorithm>
#include "pwmDefines.h"

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

class DifferentialDCMotor {

	enum status
	{
		idle_stat,
		forwardRun,
		backRun
	};

	const uint8_t idleStateValue = 0;
	const uint16_t idlePWMValue = 0;
	uint16_t pwmValue[2];
	uint16_t pwmDesiredValue[2];
	status motorPresentStatus[2];
	status motorDesiredStatus[2];
	TIM_HandleTypeDef *htim;
	uint8_t timChannel[2];

	void stop(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1, GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2);
	void left(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1, GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2);
	void right(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1, GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2);
	void makeManeuver(uint8_t motorID, uint8_t maneuver, uint16_t pwmData);
	void setPWMOnTimer(uint8_t timChannel, uint16_t pwmValue);

	void forward_R(uint16_t pwmValue);
	void forward_L(uint16_t pwmValue);
	void back_R(uint16_t pwmValue);
	void back_L(uint16_t pwmValue);
	void stop_R();
	void stop_L();
	void setIdle();

public:
	DifferentialDCMotor();
	virtual ~DifferentialDCMotor();

	void initialize(TIM_HandleTypeDef *htim, uint8_t leftTimChannel, uint8_t rightTimChannel);

	void drivingIteration();
	void stopPlatform();
	void configureDesiredPWM(uint8_t *leftMotorDirection, uint8_t *rightMotorDirection, uint16_t *leftDesiredPWM, uint16_t *rightDesiredPWM);
	uint8_t getDataInArray(uint8_t* dataBuffer);

};

#endif /* DIFFERENTIALDCMOTOR_H_ */

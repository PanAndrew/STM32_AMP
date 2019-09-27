/*
 * DCMotor.h
 *
 *  Created on: May 15, 2019
 *      Author: piotrek
 */

#ifndef DCMOTOR_H_
#define DCMOTOR_H_

#include "stdint.h"
#include "main.h"
#include <algorithm>

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define DIRECTIONS_COMMAND 0
#define LEFT_PWM_COMMAND_H 1
#define LEFT_PWM_COMMAND_L 2
#define RIGHT_PWM_COMMAND_H 3
#define RIGHT_PWM_COMMAND_L 4

#define MAX_VOLTAGE 6
#define BATTERY_VOLTAGE 12

#define MOTOR_OBJECTDATAVOLUME 5

class DCMotor {

	enum status
	{
		idle,
		forwardRun,
		backRun
	};

	uint16_t pwmValue[2];
	status motorStatus[2];
	TIM_HandleTypeDef *htim;

	void stop(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1, GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2);
	void left(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1, GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2);
	void right(GPIO_TypeDef *gpioBase_1, uint16_t gpioPin_1, GPIO_TypeDef *gpioBase_2, uint16_t gpioPin_2);
	void makeManeuver(uint8_t motorID, uint8_t maneuver, uint16_t pwmData);
	void setPWMOnTimer(TIM_HandleTypeDef *htim, uint8_t timChannel, uint16_t pwmValue);

public:
	DCMotor(TIM_HandleTypeDef *htim);
	virtual ~DCMotor();

	void forward_R(TIM_HandleTypeDef *htim, uint16_t pwmValue);
	void forward_L(TIM_HandleTypeDef *htim, uint16_t pwmValue);
	void back_R(TIM_HandleTypeDef *htim, uint16_t pwmValue);
	void back_L(TIM_HandleTypeDef *htim, uint16_t pwmValue);
	void stop_R(void);
	void stop_L(void);

	void configureDesiredPWM(uint8_t* dataBuffer);

	uint8_t getDataInArray(uint8_t* dataBuffer);

};

#endif /* DCMOTOR_H_ */

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

	void stop(GPIO_TypeDef *gpioBase, uint16_t gpioPin_1, uint16_t gpioPin_2);
	void left(GPIO_TypeDef *gpioBase, uint16_t gpioPin_1, uint16_t gpioPin_2);
	void right(GPIO_TypeDef *gpioBase, uint16_t gpioPin_1, uint16_t gpioPin_2);

public:
	DCMotor();
	virtual ~DCMotor();

	void forward_R(TIM_HandleTypeDef *htim, uint16_t pwmValue);
	void forward_L(TIM_HandleTypeDef *htim, uint16_t pwmValue);
	void back_R(TIM_HandleTypeDef *htim, uint16_t pwmValue);
	void back_L(TIM_HandleTypeDef *htim, uint16_t pwmValue);
	void stop_R(void);
	void stop_L(void);

	uint8_t getDataInArray(uint8_t* dataBuffer);

};

#endif /* DCMOTOR_H_ */

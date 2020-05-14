/*
 * TimerConfigurator.h
 *
 *  Created on: Nov 13, 2019
 *      Author: piotrek
 */

#ifndef TIMERCONFIGURATOR_H_
#define TIMERCONFIGURATOR_H_

#include "stdint.h"
#include <algorithm>
#include "unions.h"
#include "main.h"

#define TIMER_ID 0
#define PRESCALER_VALUE_H 1
#define PRESCALER_VALUE_L 2
#define ARR_VALUE_H 3
#define ARR_VALUE_L 4
#define CLOCK_DIVIDER 5

#define HTIM_IMU_ID 6
#define HTIM_DATA_SEND_ID 9
#define HTIM_LED 7

#define TIM_CONF_OBJECTDATAVOLUME 18

class TimerConfigurator {

	TIM_HandleTypeDef* htimIMU;
	TIM_HandleTypeDef* htimDataSend;
	TIM_HandleTypeDef* htimLed;

	floatUnion floatValTemp;

	void manageConfiguration(uint8_t &timerID, uint16_t &prescalerValue,
			uint16_t &ARRValue, uint8_t &clockDividerValue);

	void setConfigurationAdvTim(TIM_HandleTypeDef* htim, uint16_t &prescalerValue,
			uint16_t &ARRValue, uint8_t &clockDividerValue);

	void setConfigurationTim(TIM_HandleTypeDef* htim, uint16_t &prescalerValue,
				uint16_t &ARRValue);

	float calcFreq(uint32_t clock, uint16_t psc, uint16_t arr);
public:
	TimerConfigurator(TIM_HandleTypeDef* htim_IMU,
					TIM_HandleTypeDef* htim_DataSend,
					TIM_HandleTypeDef* htim_Led);
	virtual ~TimerConfigurator();

	void configureTimer(uint8_t* dataBuffer);
	uint8_t getDataInArray(uint8_t* dataBuffer);
};

#endif /* TIMERCONFIGURATOR_H_ */

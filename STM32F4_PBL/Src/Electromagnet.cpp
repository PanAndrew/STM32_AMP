/*
 * Electromagnet.cpp
 *
 *  Created on: Mar 22, 2020
 *      Author: piotrek
 */

#include "Electromagnet.h"

Electromagnet::Electromagnet() {
	// TODO Auto-generated constructor stub

}

Electromagnet::~Electromagnet() {
	// TODO Auto-generated destructor stub
}

void Electromagnet::configureElectromagnes(uint8_t *dataBuffer)
{
	if(dataBuffer[0] == 1)
	{
		HAL_GPIO_WritePin(GPIO_ELECTROMAGNET_GPIO_Port, GPIO_ELECTROMAGNET_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIO_ELECTROMAGNET_GPIO_Port, GPIO_ELECTROMAGNET_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
	}
}

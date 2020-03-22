/*
 * ADASManager.h
 *
 *  Created on: Mar 21, 2020
 *      Author: piotrek
 */

#ifndef ADASMANAGER_H_
#define ADASMANAGER_H_

#include "main.h"

class ADASManager {

public:
	ADASManager();
	virtual ~ADASManager();

	CAN_TxHeaderTypeDef txHeader;
	uint8_t adasFrameArray[8];

	void f119();
	void f122();
	void f124();
	void f1AF();
	void f1D0();
	void f1E8();
	void f1EF();
	void f2A2();
	void f318();
	void f32B();
	void f380();
	void f3A1();
	void f3D1();
	void f463();
};

#endif /* ADASMANAGER_H_ */

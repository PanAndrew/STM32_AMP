/*
 * Electromagnet.h
 *
 *  Created on: Mar 22, 2020
 *      Author: piotrek
 */

#ifndef ELECTROMAGNET_H_
#define ELECTROMAGNET_H_

#include "stdint.h"
#include "main.h"

class Electromagnet {


public:
	Electromagnet();
	virtual ~Electromagnet();

	void configureElectromagnes(uint8_t* dataBuffer);
};

#endif /* ELECTROMAGNET_H_ */

/*
 * unions.h
 *
 *  Created on: Jan 15, 2020
 *      Author: piotrek
 */

#ifndef UNIONS_H_
#define UNIONS_H_

union floatUnion
{
	float floatVal = 0;
	uint8_t arrVal[sizeof(float)];
};

#endif /* UNIONS_H_ */

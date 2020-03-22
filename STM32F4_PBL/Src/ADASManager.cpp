/*
 * ADASManager.cpp
 *
 *  Created on: Mar 21, 2020
 *      Author: piotrek
 */

#include "ADASManager.h"

ADASManager::ADASManager() {
	// TODO Auto-generated constructor stub

}

ADASManager::~ADASManager() {
	// TODO Auto-generated destructor stub
}

void ADASManager::f119()
{
	txHeader.StdId = 0x119;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 8;
	adasFrameArray[0] = 0x80;
	adasFrameArray[1] = 0x00;
	adasFrameArray[2] = 0x00;
	adasFrameArray[3] = 0x00;
	adasFrameArray[4] = 0x80;
	adasFrameArray[5] = 0x00;
	adasFrameArray[6] = 0x00;
	adasFrameArray[7] = 0x00;
}

void ADASManager::f122()
{
	txHeader.StdId = 0x122;
	txHeader.ExtId = 0x000;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 7;
	adasFrameArray[0] = 0x00;
	adasFrameArray[1] = 0x00;
	adasFrameArray[2] = 0x00;
	adasFrameArray[3] = 0x00;
	adasFrameArray[4] = 0x00;
	adasFrameArray[5] = 0x00;
	adasFrameArray[6] = 0x00;
}

void ADASManager::f124()
{
	txHeader.StdId = 0x124;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 8;
	adasFrameArray[0] = 0x00;
	adasFrameArray[1] = 0x00;
	adasFrameArray[2] = 0x00;
	adasFrameArray[3] = 0x00;
	adasFrameArray[4] = 0x00;
	adasFrameArray[5] = 0x00;
	adasFrameArray[6] = 0x20;
	adasFrameArray[7] = 0x00;
}

void ADASManager::f1AF()
{
	txHeader.StdId = 0x1AF;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 8;
	adasFrameArray[0] = 0x00;
	adasFrameArray[1] = 0x00;
	adasFrameArray[2] = 0x00;
	adasFrameArray[3] = 0x00;
	adasFrameArray[4] = 0x00;
	adasFrameArray[5] = 0x00;
	adasFrameArray[6] = 0x00;
	adasFrameArray[7] = 0x00;
}

void ADASManager::f1D0()
{
	txHeader.StdId = 0x1D0;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 8;
	adasFrameArray[0] = 0x00;
	adasFrameArray[1] = 0x00;
	adasFrameArray[2] = 0x00;
	adasFrameArray[3] = 0x00;
	adasFrameArray[4] = 0x00;
	adasFrameArray[5] = 0x00;
	adasFrameArray[6] = 0x00;
	adasFrameArray[7] = 0x00;
}

void ADASManager::f1E8()
{
	txHeader.StdId = 0x1E8;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 3;
	adasFrameArray[0] = 0x0D;
	adasFrameArray[1] = 0xAC;
	adasFrameArray[2] = 0x00;
}

void ADASManager::f1EF()
{
	txHeader.StdId = 0x1EF;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 8;
	adasFrameArray[0] = 0x00;
	adasFrameArray[1] = 0x00;
	adasFrameArray[2] = 0x00;
	adasFrameArray[3] = 0x80;
	adasFrameArray[4] = 0x36;
	adasFrameArray[5] = 0x46;
	adasFrameArray[6] = 0x81;
	adasFrameArray[7] = 0x1A;
}

void ADASManager::f2A2()
{
	txHeader.StdId = 0x2A2;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 8;
	adasFrameArray[0] = 0x00;
	adasFrameArray[1] = 0x00;
	adasFrameArray[2] = 0x00;
	adasFrameArray[3] = 0x00;
	adasFrameArray[4] = 0x00;
	adasFrameArray[5] = 0x00;
	adasFrameArray[6] = 0x00;
	adasFrameArray[7] = 0x00;
}

void ADASManager::f318()
{
	txHeader.StdId = 0x318;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 8;
	adasFrameArray[0] = 0x00;
	adasFrameArray[1] = 0x00;
	adasFrameArray[2] = 0x00;
	adasFrameArray[3] = 0x00;
	adasFrameArray[4] = 0x00;
	adasFrameArray[5] = 0x00;
	adasFrameArray[6] = 0x00;
	adasFrameArray[7] = 0xC0;
}

void ADASManager::f32B()
{
	txHeader.StdId = 0x32B;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 1;
	adasFrameArray[0] = 0x00;
}

void ADASManager::f380()
{
	txHeader.StdId = 0x380;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 8;
	adasFrameArray[0] = 0x00;
	adasFrameArray[1] = 0x00;
	adasFrameArray[2] = 0x00;
	adasFrameArray[3] = 0x00;
	adasFrameArray[4] = 0x00;
	adasFrameArray[5] = 0x00;
	adasFrameArray[6] = 0x00;
	adasFrameArray[7] = 0x00;
}

void ADASManager::f3A1()
{
	txHeader.StdId = 0x3A1;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 8;
	adasFrameArray[0] = 0x00;
	adasFrameArray[1] = 0x04;
	adasFrameArray[2] = 0x00;
	adasFrameArray[3] = 0x00;
	adasFrameArray[4] = 0x00;
	adasFrameArray[5] = 0x00;
	adasFrameArray[6] = 0x00;
	adasFrameArray[7] = 0x00;
}

void ADASManager::f3D1()
{
	txHeader.StdId = 0x3D1;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 8;
	adasFrameArray[0] = 0x00;
	adasFrameArray[1] = 0x00;
	adasFrameArray[2] = 0x00;
	adasFrameArray[3] = 0x00;
	adasFrameArray[4] = 0x00;
	adasFrameArray[5] = 0x00;
	adasFrameArray[6] = 0x00;
	adasFrameArray[7] = 0x50;
}

void ADASManager::f463()
{
	txHeader.StdId = 0x463;
	txHeader.ExtId = 0x000;
	txHeader.RTR= CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 3;
	adasFrameArray[0] = 0x0B;
	adasFrameArray[1] = 0x00;
	adasFrameArray[2] = 0x10;
}

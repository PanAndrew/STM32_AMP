/*
 * globalDefines.h
 *
 *  Created on: Sep 21, 2019
 *      Author: piotrek
 */

#ifndef GLOBALDEFINES_H_
#define GLOBALDEFINES_H_

#define ID_PWM 0x02
#define ID_TIM 0x03
#define ID_ACC 0x04
#define ID_GYRO 0x05
#define ID_MAG 0x06
#define ID_ENCODER 0x07
#define ID_GPS 0x08
#define ID_MINI_LIDAR 0x09
#define ID_TIME_SYNC 0x0A

#define SIZE_SET_PWM 8
#define SIZE_SET_TIM 6

#define SIZE_GET_PWM 8
#define SIZE_GET_TIM 16
#define SIZE_GET_ACC 10
#define SIZE_GET_GYRO 10
#define SIZE_GET_MAG 10
#define SIZE_GET_ENCODER 17
#define SIZE_GET_GPS 45
#define SIZE_GET_MINI_LIDAR 5
#define SIZE_GET_TIME_SYNC 4

#define IMU_NUM_OF_ELEM 100

#endif /* GLOBALDEFINES_H_ */

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
#define ID_TIME_SYNC 0xFF
#define ID_CAN_DIST 0x0B
#define ID_CAN_SPEED 0x0C
#define ID_CAN_REFLE 0x0D
#define ID_RFID 0x0E
#define ID_ULTRASOUND 0x0F
#define ID_ELECTROMAG 0x10
#define ID_TIM_MEASURE 0x11
#define ID_PACKETS_INFO 0xFE

#define SIZE_SET_PWM 8
#define SIZE_SET_TIM 6
#define SIZE_SET_ELECTROMAG 1
#define SIZE_SET_ACC 2
#define SIZE_SET_GYRO 2
#define SIZE_SET_MAG 2

#define SIZE_GET_PWM 8
#define SIZE_GET_TIM 18
#define SIZE_GET_ACC 10
#define SIZE_GET_GYRO 10
#define SIZE_GET_MAG 10
#define SIZE_GET_ENCODER 17
#define SIZE_GET_GPS 44
#define SIZE_GET_MINI_LIDAR 5
#define SIZE_GET_TIME_SYNC 4
#define SIZE_CAN_DIST 6
#define SIZE_CAN_SPEED 6
#define SIZE_CAN_REFLE 6
#define SIZE_RFID 5
#define SIZE_GET_ULTRASOUND 2
#define SIZE_GET_TIME_MEASURE 12
#define SIZE_GET_PACKETS_INFO 8

#define IMU_NUM_OF_ELEM 250

#endif /* GLOBALDEFINES_H_ */

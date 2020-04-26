/*
 * pwmDefines.h
 *
 *  Created on: Oct 1, 2019
 *      Author: piotrek
 */

#ifndef PWMDEFINES_H_
#define PWMDEFINES_H_

#define IDLE_VALUE 1500
#define MAX_DIR_PWM 500
#define BRAKE_PWM 200

#define BREAKING_TIME_MS 200
#define IDLING_TIME_MS 50

#define DIRECTION_COMMAND 0
#define PWM_COMMAND_H 1
#define PWM_COMMAND_L 2

#define SERVO_PWM_COMMAND_H 1
#define SERVO_PWM_COMMAND_L 2

#define MOTOR_PWM_COMMAND_H 3
#define MOTOR_PWM_COMMAND_L 4

#define TIME_TO_DRIVE_H 5
#define TIME_TO_DRIVE_L 6

#define MAX_QUEUE_SIZE 255
#define QUEUE_COMMAND_FLAG 7

#define SERVO_OBJECTDATAVOLUME 3
#define MOTOR_OBJECTDATAVOLUME 3
#define DIFF_MOTOR_OBJECTDATAVOLUME 5

#endif /* PWMDEFINES_H_ */

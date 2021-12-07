// Automatic_Obstacle_Avoidance.h

#ifndef _AUTOMATIC_OBSTACLE_AVOIDANCE_h
#define _AUTOMATIC_OBSTACLE_AVOIDANCE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
#include <EEPROM.h>
#include "Servo.h"
#include "Freenove_4WD_Car_for_Arduino.h"

#define OA_SERVO_CENTER					(90)
#define OA_SCAN_ANGLE_INTERVAL	50
#define OA_SCAN_ANGLE_MIN		(OA_SERVO_CENTER - OA_SCAN_ANGLE_INTERVAL)
#define OA_SCAN_ANGLE_MAX		(OA_SERVO_CENTER + OA_SCAN_ANGLE_INTERVAL)
#define OA_WAITTING_SERVO_TIME	130

#define OA_CRUISE_SPEED			(110 + oa_VoltageCompensationToSpeed)

#define OA_ROTATY_SPEED_LOW		(120 + oa_VoltageCompensationToSpeed)
#define OA_ROTATY_SPEED_NORMAL		(150 + oa_VoltageCompensationToSpeed)
#define OA_ROTATY_SPEED_HIGH		(180 + oa_VoltageCompensationToSpeed)

#define OA_TURN_SPEED_LV4			(180 + oa_VoltageCompensationToSpeed)
#define OA_TURN_SPEED_LV1			(50 + oa_VoltageCompensationToSpeed )

#define OA_BACK_SPEED_LOW			(110 + oa_VoltageCompensationToSpeed)
#define OA_BACK_SPEED_NORMAL		(150 + oa_VoltageCompensationToSpeed)
#define OA_BACK_SPEED_HIGH			(180 + oa_VoltageCompensationToSpeed)

#define OA_OBSTACLE_DISTANCE		40
#define OA_OBSTACLE_DISTANCE_LOW	15

#define OA_SPEED_OFFSET_PER_V	35

#define OA_SERVO_OFFSET_ADDR_IN_EEPROM		0

#define MAX_DISTANCE		300		//cm
#define SONIC_TIMEOUT		(MAX_DISTANCE*60)
#define SOUND_VELOCITY		340		//soundVelocity: 340m/s


extern Servo servo;
extern char servoOffset;

void servoSetup();
void setServoOffset(char offset);
void writeServo(u8 n);
void writeServoOffsetToEEPROM();
void getServoOffsetFromEEPROM();

float getSonar();
void oa_CalculateVoltageCompensation();
void updateAutomaticObstacleAvoidance();

#endif

// Freenove_4WD_Car_for_Arduino.h

#ifndef _FREENOVE_4WD_CAR_FOR_ARDUINO_h
#define _FREENOVE_4WD_CAR_FOR_ARDUINO_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define PIN_SERVO			2

#define PIN_DIRECTION_LEFT	4
#define PIN_DIRECTION_RIGHT	3
#define PIN_MOTOR_PWM_LEFT	6
#define PIN_MOTOR_PWM_RIGHT	5

#define PIN_SONIC_TRIG		7
#define	PIN_SONIC_ECHO		8

#define PIN_IRREMOTE_RECV	9

#define PIN_SPI_CE			9
#define PIN_SPI_CSN			10
#define PIN_SPI_MOSI		11
#define PIN_SPI_MISO		12
#define PIN_SPI_SCK			13

#define PIN_BATTERY			A0
#define PIN_BUZZER			A0

#define PIN_TRACKING_LEFT	A1
#define PIN_TRACKING_CENTER	A2
#define PIN_TRACKING_RIGHT	A3
#define MOTOR_PWM_DEAD		5

#define BAT_VOL_STANDARD	7.0

extern float batteryVoltage;

void pinsSetup();
void motorRun(int speedl, int speedr);

bool getBatteryVoltage();
void setBuzzer(bool flag);
void alarm(u8 beat, u8 repeat);
void resetCarAction();

#endif

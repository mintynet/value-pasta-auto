// 
// 
// 

#include "Freenove_4WD_Car_for_Arduino.h"

float batteryVoltage = 0;
bool isBuzzered = false;

void pinsSetup() {
	pinMode(PIN_DIRECTION_LEFT, OUTPUT);
	pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
	pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
	pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);

	pinMode(PIN_SONIC_TRIG, OUTPUT);// set trigPin to output mode
	pinMode(PIN_SONIC_ECHO, INPUT); // set echoPin to input mode

	pinMode(PIN_TRACKING_LEFT, INPUT); // 
	pinMode(PIN_TRACKING_RIGHT, INPUT); // 
	pinMode(PIN_TRACKING_CENTER, INPUT); // 

	setBuzzer(false);
}

void motorRun(int speedl, int speedr) {
	int dirL = 0, dirR = 0;
	if (speedl > 0) {
		dirL = 0;
	}
	else {
		dirL = 1;
		speedl = -speedl;
	}

	if (speedr > 0) {
		dirR = 1;
	}
	else {
		dirR = 0;
		speedr = -speedr;
	}

	speedl = constrain(speedl, 0, 255);
	speedr = constrain(speedr, 0, 255);

	if (abs(speedl) < MOTOR_PWM_DEAD && abs(speedr) < MOTOR_PWM_DEAD) {
		speedl = 0;
		speedr = 0;
	}

	digitalWrite(PIN_DIRECTION_LEFT, dirL);
	digitalWrite(PIN_DIRECTION_RIGHT, dirR);
	analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
	analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}


bool getBatteryVoltage() {
	if (!isBuzzered) {
		pinMode(PIN_BATTERY, INPUT);
		int batteryADC = analogRead(PIN_BATTERY);
		if (batteryADC < 614)		// 3V/12V ,Voltage read: <2.1V/8.4V
		{
			batteryVoltage = batteryADC / 1023.0 * 5.0 * 4;
			return true;
		}
	}
	return false;
}
void setBuzzer(bool flag) {
	isBuzzered = flag;
	pinMode(PIN_BUZZER, flag);
	digitalWrite(PIN_BUZZER, flag);
}
void alarm(u8 beat, u8 repeat) {
	beat = constrain(beat, 1, 9);
	repeat = constrain(repeat, 1, 255);
	for (int j = 0; j < repeat; j++) {
		for (int i = 0; i < beat; i++) {
			setBuzzer(true);
			delay(100);
			setBuzzer(false);
			delay(100);
		}
		delay(500);
	}
}
void resetCarAction() {
	motorRun(0, 0);
	setBuzzer(false);
}

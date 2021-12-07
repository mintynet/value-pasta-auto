// 
// 
// 

#include "Automatic_Obstacle_Avoidance_Mode.h"

Servo servo;
char servoOffset = 0;
int oa_VoltageCompensationToSpeed;
int distance[3];
void servoSetup() {
	getServoOffsetFromEEPROM();
	servo.attach(PIN_SERVO);
	servo.write(90 + servoOffset);
}

void setServoOffset(char offset) {
	servoOffset = offset = constrain(offset, -100, 100);
	servo.write(90 + offset);
}

void writeServo(u8 n)
{
	servo.write(90 + servoOffset);
}

void writeServoOffsetToEEPROM() {
	servo.write(90 + servoOffset);
	EEPROM.write(OA_SERVO_OFFSET_ADDR_IN_EEPROM, servoOffset);
}

void getServoOffsetFromEEPROM() {
	servoOffset = EEPROM.read(OA_SERVO_OFFSET_ADDR_IN_EEPROM);
	servoOffset = constrain(servoOffset, -10, 10);
}

float getSonar() {
	unsigned long pingTime;
	float distance;
	digitalWrite(PIN_SONIC_TRIG, HIGH); // make trigPin output high level lasting for 10��s to triger HC_SR04,
	delayMicroseconds(10);
	digitalWrite(PIN_SONIC_TRIG, LOW);
	pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait HC-SR04 returning to the high level and measure out this waitting time
	if (pingTime != 0)
		distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000; // calculate the distance according to the time
	else
		distance = MAX_DISTANCE;
	return distance; // return the distance value
}

void oa_CalculateVoltageCompensation() {
	getBatteryVoltage();
	float voltageOffset = BAT_VOL_STANDARD - batteryVoltage;
	oa_VoltageCompensationToSpeed = voltageOffset * OA_SPEED_OFFSET_PER_V;
	/*Serial.print(voltageOffset);
	Serial.print('\t');
	Serial.println(oa_VoltageCompensationToSpeed);*/
}

void updateAutomaticObstacleAvoidance() {
	int tempDistance[3][5], sumDisntance;
	static u8 cnt = 0, servoAngle = 0, lastServoAngle = 0;	//
	if (cnt == 0) {
		for (int i = 0; i < 3; i++) {
			servoAngle = OA_SCAN_ANGLE_MAX - i * OA_SCAN_ANGLE_INTERVAL + servoOffset;
			servo.write(servoAngle);
			if (lastServoAngle != servoAngle) {
				delay(OA_WAITTING_SERVO_TIME);
			}
			lastServoAngle = servoAngle;
			for (int j = 0; j < 5; j++) {
				tempDistance[i][j] = getSonar();
				delayMicroseconds(2 * SONIC_TIMEOUT);
				sumDisntance += tempDistance[i][j];
			}
			distance[i] = sumDisntance / 5;
			sumDisntance = 0;
		}
		cnt++;
	}
	else {
		for (int i = 2; i > 0; i--) {
			servoAngle = OA_SCAN_ANGLE_MAX - i * OA_SCAN_ANGLE_INTERVAL + servoOffset;
			servo.write(servoAngle);
			if (lastServoAngle != servoAngle) {
				delay(OA_WAITTING_SERVO_TIME);
			}
			lastServoAngle = servoAngle;
			for (int j = 0; j < 5; j++) {
				tempDistance[i][j] = getSonar();
				delayMicroseconds(2 * SONIC_TIMEOUT);
				sumDisntance += tempDistance[i][j];
			}
			distance[i] = sumDisntance / 5;
			sumDisntance = 0;
		}
		cnt = 0;
	}


	if (distance[1] < OA_OBSTACLE_DISTANCE) {				//Too little distance ahead
		if (distance[0] > OA_OBSTACLE_DISTANCE || distance[2] > OA_OBSTACLE_DISTANCE) {
			motorRun(-OA_BACK_SPEED_LOW, -OA_BACK_SPEED_LOW);	//Move back a little
			delay(100);
			if (distance[0] > distance[2]) {			//Left distance is greater than right distance
				motorRun(-OA_ROTATY_SPEED_LOW, OA_ROTATY_SPEED_LOW);
			}
			else {										//Right distance is greater than left distance
				motorRun(OA_ROTATY_SPEED_LOW, -OA_ROTATY_SPEED_LOW);
			}
		}
		else {											//Get into the dead corner, move back a little, then spin.
			motorRun(-OA_BACK_SPEED_HIGH, -OA_BACK_SPEED_HIGH);
			delay(100);
			motorRun(-OA_ROTATY_SPEED_NORMAL, OA_ROTATY_SPEED_NORMAL);
		}
	}
	else {												//No obstacles ahead
		if (distance[0] < OA_OBSTACLE_DISTANCE) {			//Obstacles on the left front.
			if (distance[0] < OA_OBSTACLE_DISTANCE_LOW) {	//Very close to the left front obstacle.
				motorRun(-OA_BACK_SPEED_LOW, -OA_BACK_SPEED_LOW);	//Move back
				delay(100);
			}
			motorRun(OA_TURN_SPEED_LV4, OA_TURN_SPEED_LV1);
		}
		else if (distance[2] < OA_OBSTACLE_DISTANCE) {			//Obstacles on the right front.
			if (distance[2] < OA_OBSTACLE_DISTANCE_LOW) {		//Very close to the right front obstacle.
				motorRun(-OA_BACK_SPEED_LOW, -OA_BACK_SPEED_LOW);	//Move back
				delay(100);
			}
			motorRun(OA_TURN_SPEED_LV1, OA_TURN_SPEED_LV4);
		}
		else {												//Cruising
			motorRun(OA_CRUISE_SPEED, OA_CRUISE_SPEED);
		}
	}

}

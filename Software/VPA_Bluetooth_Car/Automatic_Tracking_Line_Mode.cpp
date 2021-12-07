// 
// 
// 

#include "Automatic_Tracking_Line_Mode.h"

int tk_VoltageCompensationToSpeed;

void tk_CalculateVoltageCompensation() {
	getBatteryVoltage();
	float voltageOffset = BAT_VOL_STANDARD - batteryVoltage;
	tk_VoltageCompensationToSpeed = voltageOffset * TK_SPEED_OFFSET_PER_V;
	/*Serial.print(voltageOffset);
	Serial.print('\t');
	Serial.println(tk_VoltageCompensationToSpeed);*/
}

//The black line is detected to be high, whereas the white object (the reflected signal) is detected to be low.
//left center right -->0111
u8 getTrackingSensorVal() {
	u8 trackingSensorVal = 0;
	trackingSensorVal = (digitalRead(PIN_TRACKING_LEFT) == 1 ? 1 : 0) << 2 | (digitalRead(PIN_TRACKING_CENTER) == 1 ? 1 : 0) << 1 | (digitalRead(PIN_TRACKING_RIGHT) == 1 ? 1 : 0) << 0;
	return trackingSensorVal;
}

void updateAutomaticTrackingLine() {
	u8 trackingSensorVal = 0;
	trackingSensorVal = getTrackingSensorVal();
	//Serial.print(trackingSensorVal, BIN);
	switch (trackingSensorVal)
	{
	case 0:		//000
		motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED);
		break;
	case 7:		//111
		motorRun(TK_STOP_SPEED, TK_STOP_SPEED);
		break;
	case 1:		//001
		motorRun(TK_TURN_SPEED_LV4, TK_TURN_SPEED_LV1);
		break;
	case 3:		//011
		motorRun(TK_TURN_SPEED_LV3, TK_TURN_SPEED_LV2);
		break;
	case 2:		//010
	case 5:		//101
		motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED);
		break;
	case 6:		//110
		motorRun(TK_TURN_SPEED_LV2, TK_TURN_SPEED_LV3);
		break;
	case 4:		//100
		motorRun(TK_TURN_SPEED_LV1, TK_TURN_SPEED_LV4);
		break;
	default:
		break;
	}
	//Serial.println();
	//delay(10);
}

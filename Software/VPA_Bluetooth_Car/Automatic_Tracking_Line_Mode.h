// Automatic_Tracking_Line_Mode.h

#ifndef _AUTOMATIC_TRACKING_LINE_MODE_h
#define _AUTOMATIC_TRACKING_LINE_MODE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Freenove_4WD_Car_for_Arduino.h"


#define TK_STOP_SPEED					0
#define TK_FORWARD_SPEED				(90 + tk_VoltageCompensationToSpeed	   )
#define TK_FORWARD_SPEED_LOW			(80 + tk_VoltageCompensationToSpeed	   )
#define TK_TURN_SPEED_LV4				(180 + tk_VoltageCompensationToSpeed   )
#define TK_TURN_SPEED_LV3				(150 + tk_VoltageCompensationToSpeed   )
#define TK_TURN_SPEED_LV2				(-140 + tk_VoltageCompensationToSpeed  )
#define TK_TURN_SPEED_LV1				(-160 + tk_VoltageCompensationToSpeed  )

#define TK_SPEED_OFFSET_PER_V			30


void tk_CalculateVoltageCompensation();
void updateAutomaticTrackingLine();

#endif

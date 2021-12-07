/**********************************************************************
  Filename    : Multifunctional_Bluetooth_Remote_Car.ino
  Product     : Freenove 4WD Car for UNO
  Description : A Multifunctional Bluetooth Remote Car.
  Auther      : www.freenove.com
  Modification: 2019/08/09
**********************************************************************/
#include "Freenove_4WD_Car_for_Arduino.h"
#include "Automatic_Obstacle_Avoidance_Mode.h"
#include "Automatic_Tracking_Line_Mode.h"
#include "BluetoothOrders.h"
#include "Freenove_WS2812B_RGBLED_Controller.h"

#define UPLOAD_VOL_TIME    3000
#define COMMANDS_COUNT_MAX  8
#define INTERVAL_CHAR '#'
u32 lastUploadVoltageTime;
String inputStringBLE = "";
bool stringComplete = false;
bool ledState = false;
int paramters[COMMANDS_COUNT_MAX], paramterCount = 0;
int bleCarMode = MODE_NONE;

#define STRIP_I2C_ADDRESS  0x20
#define STRIP_LEDS_COUNT   10

u8 colorPos = 0;
u8 colorStep = 50;
u8 stripDisplayMode = 1;
u8 currentLedIndex = 0;
u16 stripDisplayDelay = 100;
u32 lastStripUpdateTime = 0;
Freenove_WS2812B_Controller strip(STRIP_I2C_ADDRESS, STRIP_LEDS_COUNT, TYPE_GRB);

void setup() {
  pinsSetup();
  Serial.begin(9600);
  servoSetup();
  while (!strip.begin());
  strip.setAllLedsColor(0xFF0000);
}

void loop() {
  if (millis() - lastUploadVoltageTime > UPLOAD_VOL_TIME) {
    upLoadVoltageToApp();
    lastUploadVoltageTime = millis();
  }
  if (stringComplete) {
    String inputCommandArray[COMMANDS_COUNT_MAX];

    String inputStringTemp = inputStringBLE;
    for (u8 i = 0; i < COMMANDS_COUNT_MAX; i++) {
      int index = inputStringTemp.indexOf(INTERVAL_CHAR);
      if (index < 0) {
        break;
      }
      paramterCount = i;  //
      inputCommandArray[i] = inputStringTemp.substring(0, index);
      inputStringTemp = inputStringTemp.substring(index + 1);
      paramters[i] = inputCommandArray[i].toInt();
    }
    stringComplete = false;
    inputStringBLE = "";

    char commandChar = inputCommandArray[0].charAt(0);
    switch (commandChar)
    {
      case ACTION_MOVE:
        if (paramterCount == 2) {
          motorRun(paramters[1], paramters[2]);
        }
        break;
      case ACTION_CAR_MODE:
        if (paramterCount == 1) {
          bleCarMode = paramters[1];
          switch (bleCarMode)
          {
            case MODE_NONE: case MODE_GRAVITY:
              resetCarAction();
              writeServo(OA_SERVO_CENTER);
              break;
            case MODE_ULTRASONIC:
              oa_CalculateVoltageCompensation();
              break;
            case MODE_TRACKING:
              tk_CalculateVoltageCompensation();
              break;
            default:
              break;
          }
        }
        break;
      case ACTION_BUZZER:
        if (paramterCount == 1) {
          setBuzzer(paramters[1]);
        }
        break;
      case ACTION_RGB:
        Serial.println(paramterCount);
        Serial.println("POOP");
        if (paramterCount == 4) {
          stripDisplayMode = paramters[1];
          switch (stripDisplayMode)
          {
            case 0:
              colorStep = 5;
              stripDisplayDelay = 100;
              break;
            case 1:
              colorStep = 50;
              stripDisplayDelay = 50;
              break;
            case 2:
              colorStep = 5;
              stripDisplayDelay = 500;
              break;
            case 3:
              break;
            case 4:
              stripDisplayDelay = 650;
              break;
            default:
              break;
          }
        }
        break;
      default:
        break;
    }
  }
  switch (bleCarMode)
  {
    case MODE_NONE: case MODE_GRAVITY:
      break;
    case MODE_ULTRASONIC:
      upLoadSonarValueToApp();
      updateAutomaticObstacleAvoidance();
      break;
    case MODE_TRACKING:
      updateAutomaticTrackingLine();
      break;
    default:
      break;
  }
  static u8 lastColor[3];
  switch (stripDisplayMode)
  {
    case 0:
      if (millis() - lastStripUpdateTime > stripDisplayDelay)
      {
        for (int i = 0; i < STRIP_LEDS_COUNT; i++) {
          strip.setLedColorData(i, strip.Wheel(colorPos + i * 25));
        }
        strip.show();
        colorPos += colorStep;
        lastStripUpdateTime = millis();
      }
      break;
    case 1:
      if (millis() - lastStripUpdateTime > stripDisplayDelay)
      {
        strip.setLedColor(currentLedIndex, strip.Wheel(colorPos));
        currentLedIndex++;
        if (currentLedIndex == STRIP_LEDS_COUNT)
        {
          currentLedIndex = 0;
          colorPos += colorStep; //
        }
        lastStripUpdateTime = millis();
      }
      break;
    case 2:
      colorPos = colorStep;
      if (millis() - lastStripUpdateTime > stripDisplayDelay)
      {
        //static bool ledState = true;
        if (ledState)
        {
          strip.setAllLedsColor(paramters[2], paramters[3], paramters[4]);
        }
        else
        {
          strip.setAllLedsColor(0x00);
        }
        ledState = !ledState;
        lastStripUpdateTime = millis();
      }
      break;
    case 3:
      if (lastColor[0] != paramters[2] || lastColor[1] != paramters[3] || lastColor[2] != paramters[4])
      {
        strip.setAllLedsColor(paramters[2], paramters[3], paramters[4]);
        lastColor[0] = paramters[2];
        lastColor[1] = paramters[3];
        lastColor[2] = paramters[4];
      }
      break;
    case 4:
      //strip.setAllLedsColor(0x00);
      if (paramters[2] == 0) {
        //side
        strip.setLedColor(1, 0x0, 0x0, 0x0);
        strip.setLedColor(2, 0x0, 0x0, 0x0);
        strip.setLedColor(3, 0x0, 0x0, 0x0);
        strip.setLedColor(6, 0x0, 0x0, 0x0);
        strip.setLedColor(7, 0x0, 0x0, 0x0);
        strip.setLedColor(8, 0x0, 0x0, 0x0);
      }
      if ((paramters[2] & 0x02) >> 1) {
        //side
        strip.setLedColor(1, 0x3f, 0x3f, 0x3f);
        strip.setLedColor(3, 0x3f, 0x3f, 0x3f);
        strip.setLedColor(6, 0x1f, 0x00, 0x00);
        strip.setLedColor(8, 0x1f, 0x00, 0x00);
      }
      if ((paramters[2] & 0x04) >> 2) {
        //dipped
        strip.setLedColor(1, 0x7f, 0x7f, 0x7f);
        strip.setLedColor(3, 0x7f, 0x7f, 0x7f);
        strip.setLedColor(6, 0x1f, 0x00, 0x00);
        strip.setLedColor(8, 0x1f, 0x00, 0x00);
      }
      if ((paramters[2] & 0x08) >> 3) {
        //main
        strip.setLedColor(1, 0xff, 0xff, 0xff);
        strip.setLedColor(3, 0xff, 0xff, 0xff);
      }
      if (paramters[2] & 0x01) {
        // stop
        strip.setLedColor(6, 0xff, 0x00, 0x00);
        strip.setLedColor(8, 0xff, 0x00, 0x00);
      }
      if (millis() - lastStripUpdateTime > stripDisplayDelay)
      {
        //static bool ledState = true;
        if (ledState)
        {
          if (paramters[3] & 0x01) {
            //left
            strip.setLedColor(4, 0xff, 0xa5, 0x00);
            strip.setLedColor(5, 0xff, 0xa5, 0x00);
          }
          if ((paramters[3] & 0x02) >> 1) {
            //right
            strip.setLedColor(0, 0xff, 0xa5, 0x00);
            strip.setLedColor(9, 0xff, 0xa5, 0x00);
          }
        } else {
          strip.setLedColor(0,0,0,0);
          strip.setLedColor(9,0,0,0);
          strip.setLedColor(4,0,0,0);
          strip.setLedColor(5,0,0,0);
        }
        ledState = !ledState;
        lastStripUpdateTime = millis();
      }
      break;
    default:
      break;
  }
}

void upLoadVoltageToApp() {
  int voltage = 0;
  if (getBatteryVoltage()) {
    voltage = batteryVoltage * 1000;
  }
  String sendString = String(ACTION_GET_VOLTAGE) + String(INTERVAL_CHAR) + String((voltage)) + String(INTERVAL_CHAR);
  Serial.println(sendString);
}

extern int distance[3];
void upLoadSonarValueToApp() {
  String sendString = String(ACTION_ULTRASONIC) + String(INTERVAL_CHAR) + String((distance[1])) + String(INTERVAL_CHAR);
  Serial.println(sendString);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputStringBLE += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

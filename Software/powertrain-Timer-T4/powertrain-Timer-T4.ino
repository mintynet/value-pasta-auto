                                                  // Teensyduino 1.53
                                                  // Arduino 1.8.13
#include <FlexCAN_T4.h>
#include <Wire.h>                                 // version 1.0
#include <Adafruit_MCP23017.h>                    // version 1.2.0
#include <ResponsiveAnalogRead.h>                 // version 1.2.1
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

Adafruit_MCP23017 mcpA;

#define DEBUG_PORT    Serial
#define BT_CAR_PORT   Serial4
#define NEXTION_PORT  Serial5

const byte ana0x01 = A0;

ResponsiveAnalogRead analog0(ana0x01,true);

const byte ledDisp = 4;
const boolean gDebug = false;

static CAN_message_t msg;

int CANBUSSPEED = 500000;
byte nextPage = 0;

IntervalTimer my100Timer;
IntervalTimer my20Timer;
IntervalTimer my2Timer;

boolean b100Hz  = 0;
boolean b20Hz   = 0;
boolean b2Hz    = 0;

boolean curDialBit0 = 0;
boolean curDialBit1 = 0;
boolean preDialBit0 = 0;
boolean preDialBit1 = 0;

const byte msgSpacing = 200;
int dialCount = 0;
boolean dialButton = 0;
String curDialDir = "   ";
uint16_t potValue = 0;
uint16_t potValue_old = 0;

long lastCanMsg = 0;
unsigned int speedMax = 194;

//100Hz Powertrain Output
const unsigned int brakeOutputIndMSG        = 0x24;
const unsigned int throttlePositionMSG      = 0x39;
const unsigned int engineRpmMSG             = 0x43;
const unsigned int powerSteeringOutIndMSG   = 0x62;
const unsigned int shiftPositionMSG         = 0x77;
//100Hz Chassis Input
const unsigned int brakeOperationMSG        = 0x1a;
const unsigned int accelerationOperationMSG = 0x2f;
const unsigned int steeringWheelPosMSG      = 0x58;
const unsigned int shiftPositionSwitchMSG   = 0x6d;
const unsigned int turnSwitchMSG            = 0x83;
const unsigned int hornSwitchMSG            = 0x98;
const unsigned int engineStartMSG           = 0x1b8;
//20Hz Powertrain Output
const unsigned int brakeOilIndMSG           = 0x146;
const unsigned int absBrakeOperationMSG     = 0x15a;
const unsigned int throttleAdjustmentMSG    = 0x16f;
const unsigned int engineCoolantTempMSG     = 0x183;
const unsigned int engineMalfunctionMSG     = 0x18d;
const unsigned int powerSteeringMalfMSG     = 0x198;
const unsigned int engineStatusMSG          = 0x19a;
const unsigned int parkingBrakeStatusMSG    = 0x1d3;
//20Hz Chassis Input
const unsigned int lightSwitchMSG           = 0x1a7;
const unsigned int lightFlashMSG            = 0x1b1;
const unsigned int parkingBrakeMSG          = 0x1c9;
//2Hz Powertrain Output
const unsigned int fuelAmountMSG            = 0x3d4;
const unsigned int batteryWarningMSG        = 0x3de;
const unsigned int ecoDrivingJudgementMSG   = 0x482;
//reset message
const unsigned int resetMSG                 = 0x280;

struct ECU_DATA {
  unsigned int mcpA;
  unsigned int mcpB;
  int engineRpmRAW;
  int speedKphRAW;
  int brakeValueRAW;
  int acceleratorValueRAW;
  int steeringValueRAW;
  byte shiftValueRAW;
  boolean engineValueRAW;
  byte turnSwitchValueRAW;
  boolean hazzardValueRAW;
  boolean hornValueRAW;
  byte lightSwValueRAW;
  byte lightFlValueRAW;
  boolean parkingValueRAW;
  byte wiperFSwValueRAW;
  byte wiperRSwValueRAW;
  byte doorLockValueRAW;
  byte lDoorSwValueRAW;
  byte rDoorSwValueRAW;
  unsigned int brakeOutputRAW;
  unsigned int throttlePositionRAW;
  int powerSteeringRAW;
  unsigned int powerSteeringTorqueRAW;
  byte shiftPositionRAW;
  String shiftPositionVal;
  byte turnSignalIndicatorRAW;
  byte hornOperationRAW;
  byte airbagActivationRAW;
  byte brakeOilIndRAW;
  byte absBrakeOperationRAW;
  byte throttleAdjustmentRAW;
  byte engineCoolantTempRAW;
  int engineCoolantTempNum;
  byte engineMalfunctionRAW;
  byte powerSteeringMalfRAW;
  byte engineStatusRAW;
  byte parkingBrakeStatusRAW;
  byte lightIndicatorRAW;
  byte frontWiperStatusRAW;
  byte frontWiperTimerRAW;
  byte rearWiperStatusRAW;
  byte doorLockStatusRAW;
  byte lDoorPositionRAW;
  byte lDoorLimitRAW;
  byte rDoorPositionRAW;
  byte rDoorLimitRAW;
  byte fuelAmountRAW;
  byte batteryWarningRAW;
  byte ecoDrivingJudgementRAW;
  byte doorDriveUnitMalfuncRAW;
  byte seatBeltSensorRAW;
  byte seatBeltAlarmRAW;
  byte bonnetOpenSwitchRAW;
  byte trunkOpenSwitchRAW;
} ecu_data, ecu_data_old;

//**************************************************
// b100Set
//**************************************************

void b100Set() {
  b100Hz = 1;
}

//**************************************************
// b20Set
//**************************************************

void b20Set() {
  b20Hz = 1;
}

//**************************************************
// b2Set
//**************************************************

void b2Set() {
  b2Hz = 1;
}

//**************************************************
// can100Hz
//**************************************************

void can100Hz() {
  //noInterrupts();
  msg.id = brakeOutputIndMSG;
  msg.len = 8;
  msg.buf[0] = ((ecu_data.brakeValueRAW >> 8) & 0xFF);
  msg.buf[1] = ((ecu_data.brakeValueRAW >> 0) & 0xFF);
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = throttlePositionMSG;
  msg.buf[0] = ((ecu_data.acceleratorValueRAW >> 8) & 0xFF);
  msg.buf[1] = ((ecu_data.acceleratorValueRAW >> 0) & 0xFF);
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = engineRpmMSG;
  msg.buf[0] = ((ecu_data.engineRpmRAW >> 8) & 0xFF);
  msg.buf[1] = ((ecu_data.engineRpmRAW >> 0) & 0xFF);
  msg.buf[2] = ((ecu_data.speedKphRAW >> 8) & 0xFF);
  msg.buf[3] = ((ecu_data.speedKphRAW >> 0) & 0xFF);
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = powerSteeringOutIndMSG;
  msg.buf[0] = ((ecu_data.steeringValueRAW >> 8) & 0xFF);
  msg.buf[1] = ((ecu_data.steeringValueRAW >> 0) & 0xFF);
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = shiftPositionMSG;
  msg.buf[0] = ecu_data.shiftPositionRAW;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  b100Hz = 0;
  digitalWrite(ledDisp,!digitalRead(ledDisp));
  //interrupts();
} // can100Hz

//**************************************************
// can20Hz
//**************************************************

void can20Hz() {
  //noInterrupts();
  msg.id = brakeOilIndMSG;
  msg.len = 8;
  msg.buf[0] = 0xff;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = absBrakeOperationMSG;
  msg.buf[0] = 0x0;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = throttleAdjustmentMSG;
  msg.buf[0] = 0x0;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = engineCoolantTempMSG;
  //msg.len = 8;
  msg.buf[0] = ecu_data.engineCoolantTempRAW;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = engineMalfunctionMSG;
  msg.buf[0] = ecu_data.engineMalfunctionRAW;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = powerSteeringMalfMSG;
  msg.buf[0] = 0x0;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = engineStatusMSG;
  msg.buf[0] = ecu_data.engineStatusRAW;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = parkingBrakeStatusMSG;
  msg.buf[0] = ecu_data.parkingBrakeStatusRAW;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  b20Hz = 0;
  digitalWrite(ledDisp,!digitalRead(ledDisp));
  //interrupts();
} // can20Hz

//**************************************************
// can2Hz
//**************************************************

void can2Hz() {
  //noInterrupts();
  msg.id = fuelAmountMSG;
  msg.len = 8;
  msg.buf[0] = ecu_data.fuelAmountRAW;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = batteryWarningMSG;
  msg.buf[0] = !ecu_data.engineStatusRAW;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = ecoDrivingJudgementMSG;
  msg.buf[0] = 0x0;
  msg.buf[1] = 0x0;
  msg.buf[2] = 0x0;
  msg.buf[3] = 0x0;
  Can0.write(msg);
  b2Hz = 0;
  digitalWrite(ledDisp,!digitalRead(ledDisp));
  //interrupts();
} // can2Hz()

//**************************************************
// canSniff()
//**************************************************

void canSniff(const CAN_message_t &msg) {
  int msgID = msg.id;
  switch (msgID) {
    //100Hz
    case brakeOperationMSG:
      ecu_data.brakeValueRAW = ((msg.buf[0] << 8) | (msg.buf[1]));
      break;
    case accelerationOperationMSG:
      ecu_data.acceleratorValueRAW = ((msg.buf[0] << 8) | (msg.buf[1]));
      break;
    case steeringWheelPosMSG:
      ecu_data.steeringValueRAW = ((msg.buf[0] << 8) | (msg.buf[1]));
      break;
    case shiftPositionSwitchMSG:
      ecu_data.shiftValueRAW = (msg.buf[0]);
      break;
    case engineStartMSG:
      ecu_data.engineValueRAW = (msg.buf[0]);
      break;
    case turnSwitchMSG:
      ecu_data.turnSwitchValueRAW = (msg.buf[0] & 0x07);
      break;
    case hornSwitchMSG:
      ecu_data.hornValueRAW = (msg.buf[0] & 0x01);
      break;
    //20Hz
    case lightSwitchMSG:
      ecu_data.lightSwValueRAW = (msg.buf[0] & 0x07);
      break;
    case lightFlashMSG:
      ecu_data.lightFlValueRAW = (msg.buf[0] & 0x01);
      break;
    case parkingBrakeMSG:
      ecu_data.parkingValueRAW = (msg.buf[0]);
      break;
    case resetMSG:
      SCB_AIRCR = 0x05FA0004;
      break;
    default:
      break;
  }
  //Serial.print("MB "); Serial.print(msg.mb);
  //Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  //Serial.print("  LEN: "); Serial.print(msg.len);
  //Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  //Serial.print(" TS: "); Serial.print(msg.timestamp);
  //Serial.print(" ID: "); Serial.print(msg.id, HEX);
  //Serial.print(" Buffer: ");
  //for ( uint8_t i = 0; i < msg.len; i++ ) {
  //  Serial.print(msg.buf[i], HEX); Serial.print(" ");
  //}
  //Serial.println();
  lastCanMsg = millis();
} // canSniff()

//**************************************************
// checkCanValues()
//**************************************************

void checkCanValues() {
  long checkTime = millis();
  if ((checkTime - lastCanMsg) > 2000) {
    ecu_data.steeringValueRAW = 0;
    ecu_data.brakeValueRAW = 0;
    ecu_data.acceleratorValueRAW = 0;
    ecu_data.engineRpmRAW = 0;
    ecu_data.speedKphRAW = 0;
    ecu_data.brakeOutputRAW = 0;
    ecu_data.throttlePositionRAW = 0;
    ecu_data.powerSteeringRAW = 0;
    ecu_data.powerSteeringTorqueRAW = 0;
    ecu_data.shiftPositionRAW = 1;
    ecu_data.shiftPositionVal = "P";
    ecu_data.turnSignalIndicatorRAW = 0;
    ecu_data.hornOperationRAW = 0;
    ecu_data.airbagActivationRAW = 0;
    ecu_data.brakeOilIndRAW = 0;
    ecu_data.absBrakeOperationRAW = 0;
    ecu_data.throttleAdjustmentRAW = 0;
    ecu_data.engineCoolantTempRAW = 0;
    ecu_data.engineCoolantTempNum = 0;
    ecu_data.engineMalfunctionRAW = 1;
    ecu_data.powerSteeringMalfRAW = 0;
    ecu_data.engineStatusRAW = 0;
    ecu_data.parkingBrakeStatusRAW = 0;
    ecu_data.lightIndicatorRAW = 0;
    ecu_data.frontWiperStatusRAW = 0;
    ecu_data.frontWiperTimerRAW = 0;
    ecu_data.rearWiperStatusRAW = 0;
    ecu_data.doorLockStatusRAW = 0;
    ecu_data.lDoorPositionRAW = 0;
    ecu_data.lDoorLimitRAW = 0;
    ecu_data.rDoorPositionRAW = 0;
    ecu_data.rDoorLimitRAW = 0;
    ecu_data.fuelAmountRAW = 0;
    ecu_data.batteryWarningRAW = 0;
    ecu_data.ecoDrivingJudgementRAW = 0;
    ecu_data.doorDriveUnitMalfuncRAW = 0;
    ecu_data.seatBeltSensorRAW = 0;
    ecu_data.seatBeltAlarmRAW = 0;
    ecu_data.bonnetOpenSwitchRAW = 0;
    ecu_data.trunkOpenSwitchRAW = 0;
    ecu_data.frontWiperStatusRAW = 0;
    ecu_data.frontWiperTimerRAW = 0;
    ecu_data.rearWiperStatusRAW = 0;
    ecu_data.doorLockStatusRAW = 0;
    ecu_data.lDoorPositionRAW = 0;
    ecu_data.lDoorLimitRAW = 0;
    ecu_data.rDoorPositionRAW = 0;
    ecu_data.rDoorLimitRAW = 0;
    ecu_data.fuelAmountRAW = 0;
    ecu_data.batteryWarningRAW = 0;
    ecu_data.ecoDrivingJudgementRAW = 0;
    ecu_data.doorDriveUnitMalfuncRAW = 0;
    ecu_data.seatBeltSensorRAW = 0;
    ecu_data.seatBeltAlarmRAW = 0;
    ecu_data.bonnetOpenSwitchRAW = 0;
    ecu_data.trunkOpenSwitchRAW = 0;
    dispPowertrain();
  }
} // checkCanValues()

//**************************************************
// updateCanValues()
//**************************************************

void updateCanValues() {
  ecu_data_old = ecu_data;
}

//**************************************************
// btSend()
//**************************************************

void btSend() {
  if ((ecu_data.lightFlValueRAW!=ecu_data_old.lightFlValueRAW)|
    (ecu_data.lightSwValueRAW!=ecu_data_old.lightSwValueRAW)|
    (ecu_data.turnSwitchValueRAW!=ecu_data_old.turnSwitchValueRAW)|
    (ecu_data.brakeValueRAW!=ecu_data_old.brakeValueRAW)) {
    int lightNum = 0;
    if (ecu_data.lightFlValueRAW) {
      lightNum = lightNum + 8;
    }
    lightNum = lightNum + (ecu_data.lightSwValueRAW << 1);
    if (ecu_data.brakeValueRAW > 0x20) {
      lightNum = lightNum + 1;
    }
    BT_CAR_PORT.print("C#4#");
    BT_CAR_PORT.print(lightNum);
    BT_CAR_PORT.print("#");
    DEBUG_PORT.print("C#4#");
    DEBUG_PORT.print(lightNum);
    DEBUG_PORT.print("#");
    if ((ecu_data.turnSwitchValueRAW & 0x04) >> 2) {
      BT_CAR_PORT.print("3");
      DEBUG_PORT.print("3");
    } else {
      BT_CAR_PORT.print(ecu_data.turnSwitchValueRAW);
      DEBUG_PORT.print(ecu_data.turnSwitchValueRAW);
    }
    BT_CAR_PORT.println("#0#");
    DEBUG_PORT.println("#0#");
  }
  if (ecu_data.hornValueRAW!=ecu_data_old.hornValueRAW) {
    if (ecu_data.hornValueRAW) {
      BT_CAR_PORT.println("D#2000#");
    } else {
      BT_CAR_PORT.println("D#0#");
    }
  }
  if (ecu_data.engineStatusRAW){
    int16_t tempSteering = 0;
    tempSteering = (int16_t)ecu_data.steeringValueRAW;
    if ((ecu_data.shiftPositionRAW!=ecu_data_old.shiftPositionRAW)|
      (ecu_data.acceleratorValueRAW!=ecu_data_old.acceleratorValueRAW)|
      (ecu_data.brakeValueRAW!=ecu_data_old.brakeValueRAW)|
      (ecu_data.parkingValueRAW!=ecu_data_old.parkingValueRAW)|
      (ecu_data.steeringValueRAW!=ecu_data_old.steeringValueRAW)) {
      int leftWheel = 0;
      int rightWheel = 0;
      if ((dialCount == 7)&(ecu_data.shiftPositionRAW == 5)) {
        // LOW GEAR
        leftWheel = (ecu_data.acceleratorValueRAW * 127 / 1024) - (ecu_data.brakeValueRAW * 127 / 1024) - (ecu_data.parkingValueRAW * 127 / 1);
        if (leftWheel < 0) {
          leftWheel = 0;
        }
        rightWheel = (ecu_data.acceleratorValueRAW * 127 / 1024) - (ecu_data.brakeValueRAW * 127 / 1024) - (ecu_data.parkingValueRAW * 127 / 1);
        if (rightWheel < 0) {
          rightWheel = 0;
        }
        ecu_data.speedKphRAW = (leftWheel + rightWheel)/2;
        ecu_data.engineRpmRAW = (ecu_data.acceleratorValueRAW * 8124 / 1024);
        if (tempSteering < -255) {
          leftWheel = -leftWheel;
        } else if (tempSteering < -64) {
          //leftWheel = leftWheel/2;
          leftWheel = 0;
        } else if (tempSteering > 255) {
          rightWheel = -rightWheel;
        } else if (tempSteering > 64) {
          //rightWheel = rightWheel/2;
          rightWheel = 0;
        }
        DEBUG_PORT.print("Steering ");
        DEBUG_PORT.println(tempSteering);
        DEBUG_PORT.print("leftWheel  ");
        DEBUG_PORT.println(leftWheel);
        DEBUG_PORT.print("rightWheel ");
        DEBUG_PORT.println(rightWheel);
        BT_CAR_PORT.print("A#");
        BT_CAR_PORT.print(leftWheel);
        BT_CAR_PORT.print("#");
        BT_CAR_PORT.print(rightWheel);
        BT_CAR_PORT.println("#");
        DEBUG_PORT.print("A#");
        DEBUG_PORT.print(leftWheel);
        DEBUG_PORT.print("#");
        DEBUG_PORT.print(rightWheel);
        DEBUG_PORT.println("#");
      } else if ((dialCount == 7)&(ecu_data.shiftPositionRAW == 4)) {
        // DRIVE GEAR
        leftWheel = (ecu_data.acceleratorValueRAW * 255 / 1024) - (ecu_data.brakeValueRAW * 255 / 1024) - (ecu_data.parkingValueRAW * 255 / 1);
        if (leftWheel < 0) {
          leftWheel = 0;
        }
        rightWheel = (ecu_data.acceleratorValueRAW * 255 / 1024) - (ecu_data.brakeValueRAW * 255 / 1024) - (ecu_data.parkingValueRAW * 255 / 1);
        if (rightWheel < 0) {
          rightWheel = 0;
        }
        ecu_data.speedKphRAW = (leftWheel + rightWheel)/2;
        ecu_data.engineRpmRAW = (ecu_data.acceleratorValueRAW * 8124 / 1024);
        if (tempSteering < -255) {
          leftWheel = -leftWheel;
        } else if (tempSteering < -64) {
          //leftWheel = leftWheel/2;
          leftWheel = 0;
        } else if (tempSteering > 255) {
          rightWheel = -rightWheel;
        } else if (tempSteering > 64) {
          //rightWheel = rightWheel/2;
          rightWheel = 0;
        }
        DEBUG_PORT.print("Steering ");
        DEBUG_PORT.println(tempSteering);
        DEBUG_PORT.print("leftWheel  ");
        DEBUG_PORT.println(leftWheel);
        DEBUG_PORT.print("rightWheel ");
        DEBUG_PORT.println(rightWheel);
        BT_CAR_PORT.print("A#");
        BT_CAR_PORT.print(leftWheel);
        BT_CAR_PORT.print("#");
        BT_CAR_PORT.print(rightWheel);
        BT_CAR_PORT.println("#");
        DEBUG_PORT.print("A#");
        DEBUG_PORT.print(leftWheel);
        DEBUG_PORT.print("#");
        DEBUG_PORT.print(rightWheel);
        DEBUG_PORT.println("#");
      } else if ((dialCount == 7)&(ecu_data.shiftPositionRAW == 2)) {
        // REVERSE GEAR
        leftWheel = (ecu_data.acceleratorValueRAW * 127 / 1024) - (ecu_data.brakeValueRAW * 127 / 1024) - (ecu_data.parkingValueRAW * 127 / 1);
        if (leftWheel < 0) {
          leftWheel = 0;
        }
        rightWheel = (ecu_data.acceleratorValueRAW * 127 / 1024) - (ecu_data.brakeValueRAW * 127 / 1024) - (ecu_data.parkingValueRAW * 127 / 1);
        if (rightWheel < 0) {
          rightWheel = 0;
        }
        ecu_data.speedKphRAW = (leftWheel + rightWheel)/2;
        ecu_data.engineRpmRAW = (ecu_data.acceleratorValueRAW * 8124 / 1024);
        if (tempSteering < -255) {
          leftWheel = -leftWheel;
        } else if (tempSteering < -64) {
          //leftWheel = leftWheel/2;
          leftWheel = 0;
        } else if (tempSteering > 255) {
          rightWheel = -rightWheel;
        } else if (tempSteering > 64) {
          //rightWheel = rightWheel/2;
          rightWheel = 0;
        }
        leftWheel = -leftWheel;
        rightWheel = -rightWheel;
        DEBUG_PORT.print("Steering ");
        DEBUG_PORT.println(tempSteering);
        DEBUG_PORT.print("leftWheel  ");
        DEBUG_PORT.println(leftWheel);
        DEBUG_PORT.print("rightWheel ");
        DEBUG_PORT.println(rightWheel);
        BT_CAR_PORT.print("A#");
        BT_CAR_PORT.print(leftWheel);
        BT_CAR_PORT.print("#");
        BT_CAR_PORT.print(rightWheel);
        BT_CAR_PORT.println("#");
        DEBUG_PORT.print("A#");
        DEBUG_PORT.print(leftWheel);
        DEBUG_PORT.print("#");
        DEBUG_PORT.print(rightWheel);
        DEBUG_PORT.println("#");
      } else {
        BT_CAR_PORT.println("A#0#0#");
        DEBUG_PORT.println("A#0#0#");
      }
    }
  }
}

//**************************************************
// setup
//**************************************************

void setup() {
  // Setup Serial port
  DEBUG_PORT.begin(500000);
  // Setup Bluetooth CAR Serial port
  BT_CAR_PORT.begin(9600);
  // Setup Nextion and reset
  NEXTION_PORT.begin(250000);
  NEXTION_PORT.print(F("sleep=0")); //Wake Up Nextion
  endNextion();
  delay(500);
  NEXTION_PORT.print(F("dim=32"));  //Change brightness 0-100
  endNextion();
  NEXTION_PORT.print(F("page 0"));  //Page 0
  endNextion();
  NEXTION_PORT.print(F("thup=1"));  //allow wakeup from sleep touch
  endNextion();
  checkNext();
  // Setup CAN
  Can0.begin();
  Can0.setBaudRate(CANBUSSPEED);
  Can0.setClock(CLK_60MHz);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  //Can0.mailboxStatus();
  while (millis()<10000) {
    // do nothing
  }
  // Setup Timer interupts
  my100Timer.begin(b100Set, 10000);  // blinkLED to run every 0.01 seconds 150000 = 0.15s
  my20Timer.begin (b20Set , 50000);  // blinkLED to run every 0.05 seconds 150000 = 0.15s
  my2Timer.begin  (b2Set  ,500000);  // blinkLED to run every 0.50 seconds 150000 = 0.15s
  // Setup led Pin
  pinMode(ledDisp,OUTPUT);
  // Setup MCP23017s
  mcpA.begin();
  for(int i=0;i<16;i++) {
    mcpA.pinMode(i,INPUT);
    mcpA.pullUp(i,HIGH);
  }
  preDialBit0 = mcpA.digitalRead(0);
  BT_CAR_PORT.println("A#0#0#");     //STOP WHEELS
  BT_CAR_PORT.println("C#2#0#0#0#"); //TURN OFF LIGHTS
  /*DEBUG_PORT.println(micros());
  can100Hz();
  DEBUG_PORT.println(micros());
  can20Hz();
  DEBUG_PORT.println(micros());
  can10Hz();
  DEBUG_PORT.println(micros());*/
  ecu_data.shiftPositionVal = 0x01;
  ecu_data.shiftPositionRAW = 0x01;
  ecu_data.engineStatusRAW = 0;
  ecu_data.engineCoolantTempRAW = 140;
  ecu_data.fuelAmountRAW = 43;
} // setup()

//**************************************************
// loop
//**************************************************

//long prevMillis = 0;
//int interval = 10;
//String gearDir = " ";

void loop() {
  Can0.events();
  checkCanValues();

  uint16_t mcpAValueRead = mcpA.readGPIOAB();
  uint16_t mcpAValue = ~mcpAValueRead;
  analog0.update();

  potValue = analog0.getValue();

  ecu_data.mcpA = mcpAValue;

  //unsigned long currentMillis = millis();
  /*if (currentMillis - prevMillis > interval) {
    Serial.print("OLD shiftValueRAW   : ");
    Serial.println(ecu_data_old.shiftValueRAW,HEX);
    Serial.print("shiftValueRAW       : ");
    Serial.println(ecu_data.shiftValueRAW,HEX);
    Serial.print("shiftPositionRAW                         : ");
    Serial.print(gearDir);
    Serial.print(" : ");
    Serial.println(ecu_data.shiftPositionRAW,HEX);
    prevMillis = currentMillis;
  }*/
  if ((ecu_data_old.shiftValueRAW == 0) & (ecu_data.shiftValueRAW == 2)) {
    // downshift
    ecu_data.shiftPositionRAW--;
    if (ecu_data.shiftPositionRAW < 1) {
      ecu_data.shiftPositionRAW = 0x01;
    }
    //gearDir = "D";    
    ecu_data_old.shiftValueRAW = ecu_data.shiftValueRAW;
  } else if ((ecu_data_old.shiftValueRAW == 0) & (ecu_data.shiftValueRAW == 1)) {
    // upshift    
    ecu_data.shiftPositionRAW++;
    if (ecu_data.shiftPositionRAW == 6) {
      ecu_data.shiftPositionRAW = 0x05;
    }
    //gearDir = "U";    
    ecu_data_old.shiftValueRAW = ecu_data.shiftValueRAW;
  } else if ((ecu_data_old.shiftValueRAW == 1) & (ecu_data.shiftValueRAW == 0)) {
    // do nothing
    ecu_data_old.shiftValueRAW = ecu_data.shiftValueRAW;
  } else if ((ecu_data_old.shiftValueRAW == 2) & (ecu_data.shiftValueRAW == 0)) {
    // do nothing
    ecu_data_old.shiftValueRAW = ecu_data.shiftValueRAW;
  }
  
  ecu_data.parkingBrakeStatusRAW = ecu_data.parkingValueRAW;
  ecu_data.engineMalfunctionRAW = !ecu_data.engineValueRAW;
  ecu_data.engineStatusRAW = ecu_data.engineValueRAW;

  if ((ecu_data.mcpA != ecu_data_old.mcpA)|(potValue != potValue_old)) {
    curDialBit0 = ((mcpAValue & 0x0001) >> 0);
    curDialBit1 = ((mcpAValue & 0x0002) >> 1);
    if ((curDialBit0 != preDialBit0) && (curDialBit0 == 1)) {
      if (curDialBit1 != curDialBit0) {
        dialCount++;
        curDialDir = " CW";
        if (dialCount > 7) {
          dialCount = 7;
        }
      } else {
        BT_CAR_PORT.println("A#0#0#");
        DEBUG_PORT.println("A#0#0#");
        dialCount--;
        curDialDir = "CCW";
        if (dialCount < 0) {
          dialCount = 0;
        }
      }
    }
    preDialBit0 = curDialBit0;
    if (((mcpAValue & 0x0004) >> 2) == 1) {
      dialButton = !dialButton;
    }
    if ((ecu_data.engineStatusRAW)&(dialCount == 1)) {
      if (dialButton == 1) {
        ecu_data.speedKphRAW = potValue * 200 / 1024;
        dialButton = 0;
      }
    } else if ((ecu_data.engineStatusRAW)&(dialCount == 2)) {
      if (dialButton == 1) {
        ecu_data.engineRpmRAW = potValue * 8124 / 1024;
        dialButton = 0;
      }
    } else if (dialCount == 3) {
      if (dialButton == 1) {
        ecu_data.engineCoolantTempRAW = potValue / 4;
        dialButton = 0;
      }
    } else if (dialCount == 4) {
      if (dialButton == 1) {
        ecu_data.fuelAmountRAW = potValue * 45 / 1024;
        dialButton = 0;
      }
    }
  
    if (gDebug) {
      DEBUG_PORT.println("************************************************************");
      DEBUG_PORT.print("mcpAValue            : ");
      if (mcpAValue < 0x10) DEBUG_PORT.print("0");
      if (mcpAValue < 0x100) DEBUG_PORT.print("0");
      if (mcpAValue < 0x1000) DEBUG_PORT.print("0");
      DEBUG_PORT.println(mcpAValue,HEX);
      DEBUG_PORT.println("************************************************************");
      DEBUG_PORT.print("Dial button          : ");
      DEBUG_PORT.println(dialButton);
      DEBUG_PORT.print("Dial           Dir   : ");
      DEBUG_PORT.print(curDialDir);
      DEBUG_PORT.print(" | counter: ");
      DEBUG_PORT.println(dialCount);
      DEBUG_PORT.print("potValue             : ");
      DEBUG_PORT.println(potValue,HEX);
      DEBUG_PORT.print("potValue_old         : ");
      DEBUG_PORT.println(potValue_old,HEX);
      DEBUG_PORT.print("engineValueRAW       : ");
      DEBUG_PORT.println(ecu_data.engineValueRAW,HEX);
      DEBUG_PORT.print("engineMalfunctionRAW : ");
      DEBUG_PORT.println(ecu_data.engineMalfunctionRAW,HEX);
      DEBUG_PORT.print("engineStatusRAW      : ");
      DEBUG_PORT.println(ecu_data.engineStatusRAW,HEX);
      DEBUG_PORT.print("speedKphRAW          : ");
      DEBUG_PORT.println(ecu_data.speedKphRAW,HEX);
      DEBUG_PORT.print("engineRpmRAW         : ");
      DEBUG_PORT.println(ecu_data.engineRpmRAW,HEX);
      DEBUG_PORT.print("fuelAmountRAW        : ");
      DEBUG_PORT.println(ecu_data.fuelAmountRAW,HEX);
      DEBUG_PORT.print("engineCoolantTempRAW : ");
      DEBUG_PORT.println(ecu_data.engineCoolantTempRAW,HEX);
    }
  }
  if ((ecu_data.engineStatusRAW)&(dialCount == 6)) {
    long tempSpeed = (ecu_data.acceleratorValueRAW * 200 / 1024) - (ecu_data.brakeValueRAW * 200 / 1024) - (ecu_data.parkingValueRAW * 200 / 1);
    if (tempSpeed < 0) {
      tempSpeed = 0;
    }
    ecu_data.speedKphRAW = tempSpeed;
    ecu_data.engineRpmRAW = (ecu_data.acceleratorValueRAW * 8124 / 1024);
    //Serial.println(ecu_data.engineRpmRAW);
  }
  btSend();
  dispNext();
  ecu_data_old.mcpA = mcpAValue;
  potValue_old = potValue;
  updateCanValues();

  if (b100Hz == 1) {
    can100Hz();
  }
  if (b20Hz == 1) {
    can20Hz();
  }
  if (b2Hz == 1) {
    can2Hz();
  }
  while (BT_CAR_PORT.available()) {
    char inChar = (char)BT_CAR_PORT.read();
//    inputStringBLE += inChar;
//    if (inChar == '\n') {
//      stringComplete = true;
//    }
  }
  checkNext();
} // loop()

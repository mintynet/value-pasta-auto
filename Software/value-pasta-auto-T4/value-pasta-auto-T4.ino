                                                  // Teensyduino 1.53
                                                  // Arduino 1.8.13
#include "mcp_minty.h"
#include <mcp_can.h>                              // version 1.5 25/09/17 from https://github.com/coryjfowler/MCP_CAN_lib modified for 10MHz SPI
#include <SPI.h>                                  // version 1.0
#include <Adafruit_NeoPixel.h>                    // version 1.1.7
#include <FlexCAN_T4.h>                           // version 2018
#include <Adafruit_MCP23017.h>                    // version 1.2.0
#include <ResponsiveAnalogRead.h>                 // version 1.2.1
byte          ecuNumber         = 3;              // 0=PT,1=CH,2=BO,3=GW
boolean       gDebug            = false;
boolean       proofDebug        = true;
boolean       firewallOpen0     = false;
boolean       firewallOpen1     = false;
boolean       firewallOpen2     = false;

const unsigned long unlockId    = 0x123;          // CAN-ID to unlock the ODB2 firewall
const unsigned long lockId      = 0x124;          // CAN-ID to lock the ODB2 firewall
byte unlockBuf[8]               = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07}; // data to lock/unlock byte 0 configures which ECU

long unsigned int   rxId;                         // Used for MCP3 received msgs
unsigned char       len         = 0;              // Used for MCP3 received msgs
unsigned char       rxBuf[8];                     // Used for MCP3 received msgs
#define             MAX_PIXELS  1                 // Number of Neopixels
#define             CAN3_INT    9                 // Set INT to pin 9
#define             CAN3_CS     10                // Set INT to pin 10
#define             CAN3_SPEED  CAN_500KBPS       // 500kbps

#define             CAN3_TX0BUF 24                // TX0 RTS Pin
//#define             CAN3_TX1BUF 25                // TX1 RTS Pin
//#define             CAN3_TX2BUF 26                // TX2 RTS Pin
//#define             CAN3_RX0BF  27                // RX0 INT Pin
//#define             CAN3_RX1BF  28                // RX1 INT Pin

MCP_CAN             CANMCP3(CAN3_CS);             // CAN3 interface using CS on digital pin 10
MCP_CAN_MINTY       CANMCP3MINTY(CAN3_CS,CAN3_TX0BUF);

unsigned int        cnt0        = 0;
unsigned int        cnt1        = 0;
unsigned int        cnt2        = 0;
unsigned int        cnt30       = 0;
unsigned int        cnt31       = 0;
unsigned int        cnt32       = 0;

// 0 Powertrain
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
// 1 Chassis
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
// 2 Body
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can2;

Adafruit_MCP23017 mcpA, mcpB;

#define DEBUG_PORT    Serial
#define BT_CAR_PORT   Serial4
#define NEXTION_PORT  Serial5

const byte NEO_PIN = 4;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(MAX_PIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

const byte ana0x01A = A0;
ResponsiveAnalogRead analog0(ana0x01A,true);
const byte ana0x02F = A1;
ResponsiveAnalogRead analog1(ana0x02F,true);
const byte ana0x058 = A2;
ResponsiveAnalogRead analog2(ana0x058,true);

IntervalTimer my100Timer;
IntervalTimer my20Timer;
IntervalTimer my10Timer;
IntervalTimer my2Timer;

static CAN_message_t msg;

const byte ledDisp = 4;

// 0 Powertrain
int CANBUSSPEED0  = 500000;
// 1 Chassis
int CANBUSSPEED1  = 500000;
// 2 Body
int CANBUSSPEED2  = 500000;
// 3 OBD2
int CANBUSSPEED3  = 500000;

byte nextPage = 0;
int dialCount = 0;
boolean dialButton = 0;
String curDialDir = "   ";
uint16_t potValue = 0;
uint16_t potValue_old = 0;

boolean b100Hz  = 0;
boolean b20Hz   = 0;
boolean b10Hz   = 0;
boolean b2Hz    = 0;

boolean curDialBit0 = 0;
boolean curDialBit1 = 0;
boolean preDialBit0 = 0;
boolean preDialBit1 = 0;
boolean curLightBit0 = 0;
boolean curLightBit1 = 0;
boolean preLightBit0 = 0;
boolean preLightBit1 = 0;
boolean curWiperFrBit0 = 0;
boolean curWiperFrBit1 = 0;
boolean preWiperFrBit0 = 0;
boolean preWiperFrBit1 = 0;

const byte msgSpacing = 200;
int lightCount = -1;
String curLightDir = "   ";
int wiperFrCount = -1;
String curWiperFrDir = "   ";

long lastCanMsg = 0;

// Powertrain
// 100Hz
const unsigned int brakeOutputIndMSG        = 0x24;
const unsigned int throttlePositionMSG      = 0x39;
const unsigned int engineRpmMSG             = 0x43;
const unsigned int powerSteeringOutIndMSG   = 0x62;
const unsigned int shiftPositionMSG         = 0x77;
// 20Hz
const unsigned int brakeOilIndMSG           = 0x146;
const unsigned int absBrakeOperationMSG     = 0x15a;
const unsigned int throttleAdjustmentMSG    = 0x16f;
const unsigned int engineCoolantTempMSG     = 0x183;
const unsigned int engineMalfunctionMSG     = 0x18d;
const unsigned int powerSteeringMalfMSG     = 0x198;
const unsigned int engineStatusMSG          = 0x19a;
const unsigned int parkingBrakeStatusMSG    = 0x1d3;
// 2Hz
const unsigned int fuelAmountMSG            = 0x3d4;
const unsigned int batteryWarningMSG        = 0x3de;
const unsigned int ecoDrivingJudgementMSG   = 0x482;

// Chassis
// 100Hz
const unsigned int brakeOperationMSG        = 0x1a;
const unsigned int accelerationOperationMSG = 0x2f;
const unsigned int steeringWheelPosMSG      = 0x58;
const unsigned int shiftPositionSwitchMSG   = 0x6d;
const unsigned int engineStartMSG           = 0x1b8;
const unsigned int turnSwitchMSG            = 0x83;
const unsigned int hornSwitchMSG            = 0x98;
// 20Hz
const unsigned int lightSwitchMSG           = 0x1a7;
const unsigned int lightFlashMSG            = 0x1b1;
const unsigned int parkingBrakeMSG          = 0x1c9;
// 10Hz
const unsigned int wiperSwitchFrontMSG      = 0x25c;
const unsigned int wiperSwitchRearMSG       = 0x271;
const unsigned int doorLockUnlockMSG        = 0x286;
const unsigned int lDoorSwWindowMSG         = 0x29c;
const unsigned int rDoorSwWindowMSG         = 0x2b1;

// Body
// 100Hz
const unsigned int turnSignalIndicatorMSG   = 0x8d;
const unsigned int hornOperationMSG         = 0xa2;
const unsigned int airbagActivationMSG      = 0xb4;
// 20Hz
const unsigned int lightIndicatorMSG        = 0x1bb;
// 10Hz
const unsigned int frontWiperStatusMSG      = 0x266;
const unsigned int rearWiperStatusMSG       = 0x27b;
const unsigned int doorLockStatusMSG        = 0x290;
const unsigned int lDoorPositionMSG         = 0x2bb;
const unsigned int rDoorPositionMSG         = 0x2a6;
// 2Hz
const unsigned int doorDriveUnitMalfuncMSG  = 0x420;
const unsigned int seatBeltSensorMSG        = 0x457;
const unsigned int seatBeltAlarmMSG         = 0x461;
const unsigned int bonnetOpenSwitchMSG      = 0x46c;
const unsigned int trunkOpenSwitchMSG       = 0x477;
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
  boolean hazardValueRAW;
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
// b10Set
//**************************************************

void b10Set() {
  b10Hz = 1;
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
  if (ecuNumber==0) {
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
  } else if (ecuNumber==1) {
    //noInterrupts();
    msg.id = brakeOperationMSG;
    msg.len = 8;
    msg.buf[0] = ((ecu_data.brakeValueRAW >> 8) & 0xFF);
    msg.buf[1] = ((ecu_data.brakeValueRAW >> 0) & 0xFF);
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = accelerationOperationMSG;
    msg.buf[0] = ((ecu_data.acceleratorValueRAW >> 8) & 0xFF);
    msg.buf[1] = ((ecu_data.acceleratorValueRAW >> 0) & 0xFF);
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = steeringWheelPosMSG;
    msg.buf[0] = ((ecu_data.steeringValueRAW >> 8) & 0xFF);
    msg.buf[1] = ((ecu_data.steeringValueRAW >> 0) & 0xFF);
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = shiftPositionSwitchMSG;
    msg.buf[0] = ecu_data.shiftValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = engineStartMSG;
    msg.buf[0] = ecu_data.engineValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = turnSwitchMSG;
    msg.buf[0] = ecu_data.turnSwitchValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = hornSwitchMSG;
    msg.buf[0] = ecu_data.hornValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    b100Hz = 0;
    digitalWrite(ledDisp,!digitalRead(ledDisp));
    //interrupts();
  } else if (ecuNumber==2) {
    //noInterrupts();
    msg.id = turnSignalIndicatorMSG;
    msg.len = 8;
    msg.buf[0] = ecu_data.turnSignalIndicatorRAW;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = hornOperationMSG;
    msg.buf[0] = ecu_data.hornValueRAW;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = airbagActivationMSG;
    msg.buf[0] = 0x0;
    Can0.write(msg);
    b100Hz = 0;
    digitalWrite(ledDisp,!digitalRead(ledDisp));
    //interrupts();
  }
} // can100Hz

//**************************************************
// can20Hz
//**************************************************

void can20Hz() {
  if (ecuNumber==0) {
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
  } else if (ecuNumber==1) {
    //noInterrupts();
    msg.id = lightSwitchMSG;
    msg.len = 8;
    msg.buf[0] = ecu_data.lightSwValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = lightFlashMSG;
    msg.buf[0] = ecu_data.lightFlValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = parkingBrakeMSG;
    msg.buf[0] = ecu_data.parkingValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    b20Hz = 0;
    digitalWrite(ledDisp,!digitalRead(ledDisp));
    //interrupts();
  } else if (ecuNumber==2) {
    //noInterrupts();
    msg.id = lightIndicatorMSG;
    msg.len = 8;
    msg.buf[0] = ecu_data.lightIndicatorRAW;
    Can0.write(msg);
    b20Hz = 0;
    digitalWrite(ledDisp,!digitalRead(ledDisp));
    //interrupts();
  }
} // can20Hz

//**************************************************
// can10Hz
//**************************************************

void can10Hz() {
  if (ecuNumber==1) {
    //noInterrupts();
    msg.id = wiperSwitchFrontMSG;
    msg.len = 8;
    msg.buf[0] = ecu_data.wiperFSwValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = wiperSwitchRearMSG;
    msg.buf[0] = ecu_data.wiperRSwValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = doorLockUnlockMSG;
    msg.buf[0] = ecu_data.doorLockValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = lDoorSwWindowMSG;
    msg.buf[0] = ecu_data.lDoorSwValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = rDoorSwWindowMSG;
    msg.buf[0] = ecu_data.rDoorSwValueRAW;
    msg.buf[1] = 0x0;
    Can0.write(msg);
    b10Hz = 0;
    digitalWrite(ledDisp,!digitalRead(ledDisp));
    //interrupts();
  } else if (ecuNumber==2) {
    //noInterrupts();
    msg.id = frontWiperStatusMSG;
    msg.len = 8;
    msg.buf[0] = ecu_data.wiperFSwValueRAW;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = rearWiperStatusMSG;
    msg.buf[0] = ecu_data.wiperRSwValueRAW;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = doorLockStatusMSG;
    msg.buf[0] = ecu_data.doorLockStatusRAW;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = lDoorPositionMSG;
    //msg.len = 2;
    if (ecu_data.lDoorLimitRAW==0) {
      msg.buf[0] = 0x00;
      msg.buf[1] = 0x01;
    } else if (ecu_data.lDoorLimitRAW==100) {
      msg.buf[0] = 0x64;
      msg.buf[1] = 0x02;
    } else {
      msg.buf[0] = ecu_data.lDoorLimitRAW;
      msg.buf[1] = 0x00;
    }
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = rDoorPositionMSG;
    //msg.len = 2;
    if (ecu_data.rDoorLimitRAW==0) {
      msg.buf[0] = 0x00;
      msg.buf[1] = 0x01;
    } else if (ecu_data.rDoorLimitRAW==100) {
      msg.buf[0] = 0x64;
      msg.buf[1] = 0x02;
    } else {
      msg.buf[0] = ecu_data.rDoorLimitRAW;
      msg.buf[1] = 0x00;
    }
    Can0.write(msg);
    b10Hz = 0;
    digitalWrite(ledDisp,!digitalRead(ledDisp));
    //interrupts();
  }
} // can10Hz()

//**************************************************
// can2Hz
//**************************************************

void can2Hz() {
  if (ecuNumber==0) {
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
  } else if (ecuNumber==2) {
    //noInterrupts();
    msg.id = doorDriveUnitMalfuncMSG;
    msg.len = 8;
    msg.buf[0] = 0x0;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = seatBeltSensorMSG;
    msg.buf[0] = 0x00;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = seatBeltAlarmMSG;
    msg.buf[0] = 0x0;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = bonnetOpenSwitchMSG;
    msg.buf[0] = ecu_data.bonnetOpenSwitchRAW;
    Can0.write(msg);
    delayMicroseconds(msgSpacing);
    msg.id = trunkOpenSwitchMSG;
    msg.buf[0] = ecu_data.trunkOpenSwitchRAW;
    Can0.write(msg);
    b2Hz = 0;
    digitalWrite(ledDisp,!digitalRead(ledDisp));
    //interrupts();
  }
} // can2Hz()

//**************************************************
// canSniff()
//**************************************************

void canSniff(const CAN_message_t &msg) {
  if (ecuNumber==0) {
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
  } else if (ecuNumber==1) {
    int msgID = msg.id;
    switch (msgID) {
      //100Hz
      case brakeOutputIndMSG:
        ecu_data.brakeOutputRAW = ((msg.buf[0] << 8) | (msg.buf[1]));
        break;
      case throttlePositionMSG:
        ecu_data.throttlePositionRAW = ((msg.buf[0] << 8) | (msg.buf[1]));
        break;
      case engineRpmMSG:
        ecu_data.engineRpmRAW = ((msg.buf[0] << 8) | (msg.buf[1]));
        ecu_data.speedKphRAW = ((msg.buf[2] << 8) | (msg.buf[3]));
        break;
      case powerSteeringOutIndMSG:
        ecu_data.powerSteeringRAW = ((msg.buf[0] << 8) | (msg.buf[1]));
        ecu_data.powerSteeringTorqueRAW = ((msg.buf[2] << 8) | (msg.buf[3]));
        break;
      case shiftPositionMSG:
        ecu_data.shiftPositionRAW = (msg.buf[0]);
        switch(ecu_data.shiftPositionRAW) {
          case 1:
            ecu_data.shiftPositionVal = "P";
            break;
          case 2:
            ecu_data.shiftPositionVal = "R";
            break;
          case 3:
            ecu_data.shiftPositionVal = "N";
            break;
          case 4:
            ecu_data.shiftPositionVal = "D";
            break;
          case 5:
            ecu_data.shiftPositionVal = "L";
            break;
          default:
            //ecu_data.shiftPositionVal = "?";
            break; 
        }
        break;
      case turnSignalIndicatorMSG:
        ecu_data.turnSignalIndicatorRAW = (msg.buf[0]);
        break;
      case hornOperationMSG:
        ecu_data.hornOperationRAW = (msg.buf[0]);
        mcpB.digitalWrite(7,ecu_data.hornOperationRAW & 0x01);
        break;
      case airbagActivationMSG:
        ecu_data.airbagActivationRAW = (msg.buf[0]);
        break;
      //20Hz
      case brakeOilIndMSG:
        ecu_data.brakeOilIndRAW = (msg.buf[0]);
        break;
      case absBrakeOperationMSG:
        ecu_data.absBrakeOperationRAW = (msg.buf[0]);
        break;
      case throttleAdjustmentMSG:
        ecu_data.throttleAdjustmentRAW = (msg.buf[0]);
        break;
      case engineCoolantTempMSG:
        ecu_data.engineCoolantTempRAW = (msg.buf[0]);
        ecu_data.engineCoolantTempNum = ecu_data.engineCoolantTempRAW - 40;
        break;
      case engineMalfunctionMSG:
        ecu_data.engineMalfunctionRAW = (msg.buf[0]);
        break;
      case powerSteeringMalfMSG:
        ecu_data.powerSteeringMalfRAW = (msg.buf[0]);
        break;
      case engineStatusMSG:
        ecu_data.engineStatusRAW = (msg.buf[0] & 0x01);
        mcpB.digitalWrite(14,ecu_data.engineStatusRAW);
        break;
      case parkingBrakeStatusMSG:
        ecu_data.parkingBrakeStatusRAW = (msg.buf[0] & 0x01);
        mcpB.digitalWrite(15,ecu_data.parkingBrakeStatusRAW);
        break;
      case lightIndicatorMSG:
        ecu_data.lightIndicatorRAW = (msg.buf[0]);
        break;
      //10Hz
      case frontWiperStatusMSG:
        ecu_data.frontWiperStatusRAW = ((msg.buf[1] & 0x0f));
        ecu_data.frontWiperTimerRAW = ((((msg.buf[1] << 8) | (msg.buf[0])) & 0xfff0) >> 7);
        break;
      case rearWiperStatusMSG:
        ecu_data.rearWiperStatusRAW = (msg.buf[0]);
        break;
      case doorLockStatusMSG:
        ecu_data.doorLockStatusRAW = (msg.buf[0]);
        if (((ecu_data.doorLockStatusRAW & 0x04) >> 2) & ((ecu_data_old.doorLockStatusRAW & 0x01) >> 0)) {
          ecu_data.doorLockValueRAW = ecu_data.doorLockValueRAW & 0xfe;
        } else if (((ecu_data.doorLockStatusRAW & 0x08) >> 3) & ((ecu_data_old.doorLockStatusRAW & 0x02) >> 1)) {
          ecu_data.doorLockValueRAW = ecu_data.doorLockValueRAW & 0xfd;
        }
        break;
      case lDoorPositionMSG:
        ecu_data.lDoorPositionRAW = (msg.buf[0]);
        ecu_data.lDoorLimitRAW = (msg.buf[1]);
        break;
      case rDoorPositionMSG:
        ecu_data.rDoorPositionRAW = (msg.buf[0]);
        ecu_data.rDoorLimitRAW = (msg.buf[1]);
        break;
      //2Hz
      case fuelAmountMSG:
        ecu_data.fuelAmountRAW = (msg.buf[0]);
        break;
      case batteryWarningMSG:
        ecu_data.batteryWarningRAW = (msg.buf[0]);
        break;
      case ecoDrivingJudgementMSG:
        ecu_data.ecoDrivingJudgementRAW = (msg.buf[0]);
        break;
      case doorDriveUnitMalfuncMSG:
        ecu_data.doorDriveUnitMalfuncRAW = (msg.buf[0]);
        break;
      case seatBeltSensorMSG:
        ecu_data.seatBeltSensorRAW = (msg.buf[0]);
        break;
      case seatBeltAlarmMSG:
        ecu_data.seatBeltAlarmRAW = (msg.buf[0]);
        break;
      case bonnetOpenSwitchMSG:
        ecu_data.bonnetOpenSwitchRAW = (msg.buf[0]);
        break;
      case trunkOpenSwitchMSG:
        ecu_data.trunkOpenSwitchRAW = (msg.buf[0]);
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
  } else if (ecuNumber==2) {
    int msgID = msg.id;
    switch (msgID) {
      //100Hz
      case turnSwitchMSG:
        ecu_data.turnSwitchValueRAW = (msg.buf[0] & 0x07);
        break;
      case hornSwitchMSG:
        ecu_data.hornValueRAW = (msg.buf[0] & 0x01);
        break;
      case brakeOperationMSG:
        ecu_data.brakeValueRAW = ((msg.buf[0] << 8) | (msg.buf[1]));
        break;
      //20Hz
      case lightSwitchMSG:
        ecu_data.lightSwValueRAW = (msg.buf[0] & 0x07);
        break;
      case lightFlashMSG:
        ecu_data.lightFlValueRAW = (msg.buf[0] & 0x01);
        break;
      //10Hz
      case wiperSwitchFrontMSG:
        ecu_data.wiperFSwValueRAW = (msg.buf[0]);
        break;
      case wiperSwitchRearMSG:
        ecu_data.wiperRSwValueRAW = (msg.buf[0]);
        break;
      case doorLockUnlockMSG:
        ecu_data.doorLockValueRAW = (msg.buf[0]);
        break;
      case lDoorSwWindowMSG:
        ecu_data.lDoorSwValueRAW = (msg.buf[0]);
        break;
      case rDoorSwWindowMSG:
        ecu_data.rDoorSwValueRAW = (msg.buf[0]);
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
  }
} // canSniff()

//**************************************************
// canSniff0() Powertrain
//**************************************************

void canSniff0(const CAN_message_t &msg0) {
  if (gDebug) DEBUG_PORT.println(F("RX CAN0"));
  switch (msg0.id) {
    case brakeOutputIndMSG:       //0x024 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case throttlePositionMSG:     //0x039 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case engineRpmMSG:            //0x043 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case powerSteeringOutIndMSG:  //0x062 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case shiftPositionMSG:        //0x077 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case brakeOilIndMSG:          //0x146 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case 0x150:                   //0x150 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case absBrakeOperationMSG:    //0x15a (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case throttleAdjustmentMSG:   //0x16f (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case 0x179:                   //0x179 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case engineCoolantTempMSG:    //0x183 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case engineMalfunctionMSG:    //0x18d (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case powerSteeringMalfMSG:    //0x198 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case engineStatusMSG:         //0x19a (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case 0x1a2:                   //0x1a2 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case 0x1ad:                   //0x1ad (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case parkingBrakeStatusMSG:   //0x1d3 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case 0x39e:                   //0x39e (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case 0x3a9:                   //0x3a9 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case 0x3b3:                   //0x3b3 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case 0x3bd:                   //0x3bd (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case 0x3c7:                   //0x3c7 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case fuelAmountMSG:           //0x3d4 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case batteryWarningMSG:       //0x3de (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case 0x42b:                   //0x42b (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    case ecoDrivingJudgementMSG:  //0x482 (PT_TO_CS)
      // 1
      Can1.write(msg0);
      break;
    default:
      break;
  }
  long millis0 = millis();
  if((millis0>20000)&(millis0<21001)&(proofDebug)) {
    cnt0++;
  }
  if (firewallOpen0) {
    if(gDebug) {
      DEBUG_PORT.println(F("CAN0 to CAN3"));
    }
    //msg0.buf[5]=0xff;
    //byte sndStat = CANMCP3.sendMsgBuf(msg0.id,0,msg0.len,msg0.buf);
    byte sndStat = CANMCP3MINTY.sendTX0(msg0.id,msg0.len,msg0.buf,1);
    if((millis0>20000)&(millis0<21001)&(proofDebug)&(sndStat==CAN_OK)) {
      cnt30++;
    }
  }
  pixels.setPixelColor(0, pixels.Color(150,0,0));
  pixels.show();
  //digitalWrite(ledDisp,!digitalRead(ledDisp));
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
} // canSniff0()

//**************************************************
// canSniff1() Chassis
//**************************************************

void canSniff1(const CAN_message_t &msg1) {
  if (gDebug) DEBUG_PORT.println(F("RX CAN1"));
  switch (msg1.id) {
    case brakeOperationMSG:       //0x01a (CS_TO_AL) == CS to PT & BD
      // 0
      Can0.write(msg1);
      // 2
      Can2.write(msg1);
      break;
    case accelerationOperationMSG://0x02f (CS_TO_AL) == CS to PT & BD
      // 0
      Can0.write(msg1);
      break;
    case steeringWheelPosMSG:     //0x058 (CS_TO_AL) == CS to PT & BD
      // 0
      Can0.write(msg1);
      break;
    case shiftPositionSwitchMSG:  //0x06d (CS_TO_AL) == CS to PT & BD
      // 0
      Can0.write(msg1);
      break;
    case engineStartMSG:          //0x1b8 (CS_TO_AL) == CS to PT & BD
      // 0
      Can0.write(msg1);
      break;
    case turnSwitchMSG:           //0x083 (CS_TO_AL) == CS to PT & BD
      // 0
      Can0.write(msg1);
      // 2
      Can2.write(msg1);
      break;
    case hornSwitchMSG:           //0x098 (CS_TO_AL) == CS to PT & BD
      // 0
      Can0.write(msg1);
      // 2
      Can2.write(msg1);
      break;
    case lightSwitchMSG:          //0x1a7 (CS_TO_AL) == CS to PT & BD
      // 0
      Can0.write(msg1);
      // 2
      Can2.write(msg1);
      break;
    case lightFlashMSG:           //0x1b1 (CS_TO_AL) == CS to PT & BD
      // 0
      Can0.write(msg1);
      // 2
      Can2.write(msg1);
      break;
    case parkingBrakeMSG:         //0x1c9 (CS_TO_AL) == CS to PT & BD
      // 0
      Can0.write(msg1);
      // 2
      Can2.write(msg1);
      break;
    case wiperSwitchFrontMSG:     //0x25c (CS_TO_AL) == CS to PT & BD
      // 2
      Can2.write(msg1);
      break;
    case wiperSwitchRearMSG:      //0x271 (CS_TO_AL) == CS to PT & BD
      // 2
      Can2.write(msg1);
      break;
    case doorLockUnlockMSG:       //0x286 (CS_TO_AL) == CS to PT & BD
      // 2
      Can2.write(msg1);
      break;
    case lDoorSwWindowMSG:        //0x29c (CS_TO_AL) == CS to PT & BD
      // 2
      Can2.write(msg1);
      break;
    case rDoorSwWindowMSG:        //0x2b1 (CS_TO_AL) == CS to PT & BD
      // 2
      Can2.write(msg1);
      break;
    default:
      break;
  }
  long millis1 = millis();
  if((millis1>20000)&(millis1<21001)&(proofDebug)) {
    cnt1++;
  }
  if (firewallOpen1) {
    if(gDebug) {
      DEBUG_PORT.println(F("CAN1 to CAN3"));
    }
    //msg1.buf[6]=0xff;
    //byte sndStat = CANMCP3.sendMsgBuf(msg1.id,0,msg1.len,msg1.buf);
    byte sndStat = CANMCP3MINTY.sendTX0(msg1.id,msg1.len,msg1.buf,1);
    if((millis1>20000)&(millis1<21001)&(proofDebug)&(sndStat==CAN_OK)) {
      cnt31++;
    }
  }
  pixels.setPixelColor(0, pixels.Color(0,150,0));
  pixels.show();
  //digitalWrite(ledDisp,!digitalRead(ledDisp));
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
} // canSniff1()

//**************************************************
// canSniff2() Body
//**************************************************

void canSniff2(const CAN_message_t &msg2) {
  if (gDebug) DEBUG_PORT.println(F("RX CAN2"));
  switch (msg2.id) {
    case turnSignalIndicatorMSG:  //0x08d (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case hornOperationMSG:        //0x0a2 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case airbagActivationMSG:     //0x0b4 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case lightIndicatorMSG:       //0x1bb (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case frontWiperStatusMSG:     //0x266 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case rearWiperStatusMSG:      //0x27b (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case doorLockStatusMSG:       //0x290 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case lDoorPositionMSG:        //0x2bb (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case rDoorPositionMSG:        //0x2a6 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case 0x3e9:                   //0x3e9 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case 0x3f4:                   //0x3f4 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case 0x3ff:                   //0x3ff (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case doorDriveUnitMalfuncMSG: //0x420 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case 0x436:                   //0x436 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case 0x441:                   //0x441 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case 0x44c:                   //0x44c (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case seatBeltSensorMSG:       //0x457 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case seatBeltAlarmMSG:        //0x461 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case bonnetOpenSwitchMSG:     //0x46c (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    case trunkOpenSwitchMSG:      //0x477 (BD_TO_CS)
      // 1
      Can1.write(msg2);
      break;
    default:
      break;
  }
  long millis2 = millis();
  if((millis2>20000)&(millis2<21001)&(proofDebug)) {
    cnt2++;
  }
  if (firewallOpen2) {
    if(gDebug) {
      DEBUG_PORT.println(F("CAN2 to CAN3"));
    }
    //msg2.buf[7]=0xff;
    //byte sndStat = CANMCP3.sendMsgBuf(msg2.id,0,msg2.len,msg2.buf);
    byte sndStat = CANMCP3MINTY.sendTX0(msg2.id,msg2.len,msg2.buf,1);
    if((millis2>20000)&(millis2<21001)&(proofDebug)&(sndStat==CAN_OK)) {
      cnt32++;
    }
  }
  pixels.setPixelColor(0, pixels.Color(0,0,150));
  pixels.show();
  //digitalWrite(ledDisp,!digitalRead(ledDisp));
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
} // canSniff2()

//**************************************************
// canSniff3() OBD2
//**************************************************

void canSniff3() {
  int checkno = 0;
  CAN_message_t msg;
  if(!digitalRead(CAN3_INT)) {
    CANMCP3.readMsgBuf(&rxId, &len, rxBuf);
    checkno = rxBuf[0];
    switch (rxId) {
      case unlockId:
        if((rxBuf[1] == unlockBuf[1])&(rxBuf[2] == unlockBuf[2])&(rxBuf[3] == unlockBuf[3])&(rxBuf[4] == unlockBuf[4])&(rxBuf[5] == unlockBuf[5])&(rxBuf[6] == unlockBuf[6])&(rxBuf[7] == unlockBuf[7])) {
          if((checkno&0x01)>>0) {
            firewallOpen0 = true;
            DEBUG_PORT.println(F("Firewall 0\tOpen"));
          }
          if((checkno&0x02)>>1) {
            firewallOpen1 = true;
            DEBUG_PORT.println(F("Firewall 1\tOpen"));
          }
          if((checkno&0x04)>>2) {
            firewallOpen2 = true;
            DEBUG_PORT.println(F("Firewall 2\tOpen"));
          }
        }
        break;
      case lockId:
        if((rxBuf[1] == unlockBuf[1])&(rxBuf[2] == unlockBuf[2])&(rxBuf[3] == unlockBuf[3])&(rxBuf[4] == unlockBuf[4])&(rxBuf[5] == unlockBuf[5])&(rxBuf[6] == unlockBuf[6])&(rxBuf[7] == unlockBuf[7])) {
          if((checkno&0x01)>>0) {
            firewallOpen0 = false;
            DEBUG_PORT.println(F("Firewall 0\tClosed"));         
          }
          if((checkno&0x02)>>1) {
            firewallOpen1 = false;
            DEBUG_PORT.println(F("Firewall 1\tClosed"));         
          }
          if((checkno&0x04)>>2) {
            firewallOpen2 = false;
            DEBUG_PORT.println(F("Firewall 2\tClosed"));         
          }
        }
        break;
      case resetMSG:
        if((checkno&0x01)>>0) {
          msg.id = resetMSG;
          msg.len = 0;
          Can0.write(msg);
        }
        if((checkno&0x02)>>1) {
          msg.id = resetMSG;
          msg.len = 0;
          Can1.write(msg);
        }
        if((checkno&0x04)>>2) {
          msg.id = resetMSG;
          msg.len = 0;
          Can2.write(msg);
        }
        if((checkno&0x08)>>3) {
          delayMicroseconds(500);
          SCB_AIRCR = 0x05FA0004;
        }
        break;
      default:
        break;
    }
  }
} // canSniff3()

//**************************************************
// serialMenu()
//**************************************************

static void serialMenu() {
  int checkno = 0;
  static char cmdbuf[8];
  CAN_message_t msg;
  if (DEBUG_PORT.available()) {
    for (int i=0; i < 8; i++) {
      char ser = DEBUG_PORT.read();
      cmdbuf[i] = ser;
      if (ser == '\r') {
        checkno = cmdbuf[1];
        if (checkno >= '0' && checkno <= '9') {
          checkno = checkno - '0';
        } else if (checkno >= 'A' && checkno <= 'F') {
          checkno = checkno - 'A' + 10;
        } else if (checkno >= 'a' && checkno <= 'f') {
          checkno = checkno - 'a' + 10;
        }
        switch (cmdbuf[0]) {
          case 'd':     // toggle DEBUG
            gDebug=!gDebug;
            break;
          case 'h':     // help
            DEBUG_PORT.println(F("HELP FUNCTIONS"));
            DEBUG_PORT.println();
            DEBUG_PORT.println(F("f\tToggle All firewalls"));
            DEBUG_PORT.println(F("f1\tToggle PT  firewall"));
            DEBUG_PORT.println(F("f2\tToggle CH  firewall"));
            DEBUG_PORT.println(F("f4\tToggle BO  firewall"));
            DEBUG_PORT.println(F("\tadd numbers to change multiple"));           
            DEBUG_PORT.println(F("r\tReboot All ECUs"));
            DEBUG_PORT.println(F("r1\tReboot PT  ECU"));
            DEBUG_PORT.println(F("r2\tReboot CH  ECU"));
            DEBUG_PORT.println(F("r4\tReboot BO  ECU"));
            DEBUG_PORT.println(F("r8\tReboot GW  ECU"));
            DEBUG_PORT.println(F("\tadd numbers to reboot multiple"));           
            DEBUG_PORT.println(F("d\tToggle Debug"));           
            break;
          case 'r':     // reboot ECUs
            if (cmdbuf[1] == '\r') {
                msg.id = resetMSG;
                msg.len = 0x0;
                Can0.write(msg);
                delayMicroseconds(100);
                Can1.write(msg);
                delayMicroseconds(100);
                Can2.write(msg);
                delayMicroseconds(100);
                SCB_AIRCR = 0x05FA0004;
            } else {
              if ((checkno&0x1)>>0) {
                msg.id = resetMSG;
                msg.len = 0x0;
                Can0.write(msg);
              }
              if ((checkno&0x2)>>1) {
                msg.id = resetMSG;
                msg.len = 0x0;
                Can1.write(msg);
              }
              if ((checkno&0x4)>>2) {
                msg.id = resetMSG;
                msg.len = 0x0;
                Can2.write(msg);
              }
              if ((checkno&0x8)>>3) {
                delayMicroseconds(250);
                SCB_AIRCR = 0x05FA0004;
              }
            }
            break;
          case 'f':     // firewall toggle
            if(cmdbuf[1] == '\r') {
              firewallOpen0 = !firewallOpen0;
              firewallOpen1 = !firewallOpen1;
              firewallOpen2 = !firewallOpen2;
            } else {
              if((checkno & 0x01)>>0) {
                firewallOpen0 = !firewallOpen0;
              }
              if((checkno & 0x02)>>1) {
                firewallOpen1 = !firewallOpen1;
              }
              if((checkno & 0x04)>>2) {
                firewallOpen2 = !firewallOpen2;
              }
            }
            if (firewallOpen0) {
              DEBUG_PORT.println(F("Firewall 0\tOpen"));
            } else {
              DEBUG_PORT.println(F("Firewall 0\tClosed"));
            }
            if (firewallOpen1) {
              DEBUG_PORT.println(F("Firewall 1\tOpen"));
            } else {
              DEBUG_PORT.println(F("Firewall 1\tClosed"));
            }
            if (firewallOpen2) {
              DEBUG_PORT.println(F("Firewall 2\tOpen"));
            } else {
              DEBUG_PORT.println(F("Firewall 2\tClosed"));
            }
            break;
          default:
            break;
        }
      }
    delayMicroseconds(50);
    }
  }
} //serialMenu()

//**************************************************
// checkCanValues()
//**************************************************

void checkCanValues() {
  long checkTime = millis();
  if ((checkTime - lastCanMsg) > 2000) {
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
    ecu_data.engineMalfunctionRAW = 0;
    ecu_data.powerSteeringMalfRAW = 0;
    ecu_data.engineStatusRAW = 0;
    ecu_data.parkingBrakeStatusRAW = 0;
    ecu_data.lightIndicatorRAW = 0;
    ecu_data.frontWiperStatusRAW = 0;
    ecu_data.frontWiperTimerRAW = 0;
    ecu_data.rearWiperStatusRAW = 0;
    ecu_data.doorLockStatusRAW = 0;
    ecu_data.frontWiperStatusRAW = 0;
    ecu_data.frontWiperTimerRAW = 0;
    ecu_data.rearWiperStatusRAW = 0;
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
    BT_CAR_PORT.print(F("C#4#"));
    BT_CAR_PORT.print(lightNum);
    BT_CAR_PORT.print(F("#"));
    DEBUG_PORT.print(F("C#4#"));
    DEBUG_PORT.print(lightNum);
    DEBUG_PORT.print(F("#"));
    if ((ecu_data.turnSwitchValueRAW & 0x04) >> 2) {
      BT_CAR_PORT.print(F("3"));
      DEBUG_PORT.print(F("3"));
    } else {
      BT_CAR_PORT.print(ecu_data.turnSwitchValueRAW);
      DEBUG_PORT.print(ecu_data.turnSwitchValueRAW);
    }
    BT_CAR_PORT.println(F("#0#"));
    DEBUG_PORT.println(F("#0#"));
  }
  if (ecu_data.hornValueRAW!=ecu_data_old.hornValueRAW) {
    if (ecu_data.hornValueRAW) {
      BT_CAR_PORT.println(F("D#2000#"));
    } else {
      BT_CAR_PORT.println(F("D#0#"));
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
        DEBUG_PORT.print(F("Steering "));
        DEBUG_PORT.println(tempSteering);
        DEBUG_PORT.print(F("leftWheel  "));
        DEBUG_PORT.println(leftWheel);
        DEBUG_PORT.print(F("rightWheel "));
        DEBUG_PORT.println(rightWheel);
        BT_CAR_PORT.print(F("A#"));
        BT_CAR_PORT.print(leftWheel);
        BT_CAR_PORT.print(F("#"));
        BT_CAR_PORT.print(rightWheel);
        BT_CAR_PORT.println(F("#"));
        DEBUG_PORT.print(F("A#"));
        DEBUG_PORT.print(leftWheel);
        DEBUG_PORT.print(F("#"));
        DEBUG_PORT.print(rightWheel);
        DEBUG_PORT.println(F("#"));
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
        DEBUG_PORT.print(F("Steering "));
        DEBUG_PORT.println(tempSteering);
        DEBUG_PORT.print(F("leftWheel  "));
        DEBUG_PORT.println(leftWheel);
        DEBUG_PORT.print(F("rightWheel "));
        DEBUG_PORT.println(rightWheel);
        BT_CAR_PORT.print(F("A#"));
        BT_CAR_PORT.print(leftWheel);
        BT_CAR_PORT.print(F("#"));
        BT_CAR_PORT.print(rightWheel);
        BT_CAR_PORT.println(F("#"));
        DEBUG_PORT.print(F("A#"));
        DEBUG_PORT.print(leftWheel);
        DEBUG_PORT.print(F("#"));
        DEBUG_PORT.print(rightWheel);
        DEBUG_PORT.println(F("#"));
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
        DEBUG_PORT.print(F("Steering "));
        DEBUG_PORT.println(tempSteering);
        DEBUG_PORT.print(F("leftWheel  "));
        DEBUG_PORT.println(leftWheel);
        DEBUG_PORT.print(F("rightWheel "));
        DEBUG_PORT.println(rightWheel);
        BT_CAR_PORT.print(F("A#"));
        BT_CAR_PORT.print(leftWheel);
        BT_CAR_PORT.print(F("#"));
        BT_CAR_PORT.print(rightWheel);
        BT_CAR_PORT.println(F("#"));
        DEBUG_PORT.print(F("A#"));
        DEBUG_PORT.print(leftWheel);
        DEBUG_PORT.print(F("#"));
        DEBUG_PORT.print(rightWheel);
        DEBUG_PORT.println(F("#"));
      } else {
        BT_CAR_PORT.println(F("A#0#0#"));
        DEBUG_PORT.println(F("A#0#0#"));
      }
    }
  }
}

//**************************************************
// setup
//**************************************************

void setup() {
  if (ecuNumber==0) {
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
    Can0.setBaudRate(CANBUSSPEED0);
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
    BT_CAR_PORT.println(F("A#0#0#"));     //STOP WHEELS
    BT_CAR_PORT.println(F("C#2#0#0#0#")); //TURN OFF LIGHTS
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
  } else if (ecuNumber==1) {
    // Setup Serial port
    DEBUG_PORT.begin(500000);
    // Setup Nextion and reset
    NEXTION_PORT.begin(250000);
    NEXTION_PORT.print(F("sleep=0")); //Wake Up Nextion
    endNextion();
    delay(500);
    NEXTION_PORT.print(F("dim=32"));  //Change brightness 0-100
    endNextion();
    NEXTION_PORT.print(F("page 0"));  //Page 0
    endNextion();
    NEXTION_PORT.print(F("thup=1"));  //allow wakeup from sleep
    endNextion();
    checkNext();
    // Setup CAN
    Can0.begin();
    Can0.setClock(CLK_60MHz);
    Can0.setBaudRate(CANBUSSPEED0);
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
    my10Timer.begin (b10Set ,100000);  // blinkLED to run every 0.10 seconds 150000 = 0.15s
    // Setup led Pin
    pinMode(ledDisp,OUTPUT);
    // Setup MCP23017s
    mcpA.begin();
    mcpB.begin(1);
    for(int i=0;i<16;i++) {
      mcpA.pinMode(i,INPUT);
      mcpB.pinMode(i,INPUT);
      mcpA.pullUp(i,HIGH);
      mcpB.pullUp(i,HIGH);
    }
    mcpB.pinMode(7,OUTPUT);
    mcpB.pinMode(14,OUTPUT);
    mcpB.pinMode(15,OUTPUT);
    mcpB.digitalWrite(7,LOW);
    mcpB.digitalWrite(14,LOW);
    mcpB.digitalWrite(15,LOW);
    preLightBit0 = mcpA.digitalRead(0);
    preWiperFrBit0 = mcpA.digitalRead(8);
    ecu_data.engineValueRAW = 0;
    ecu_data.parkingValueRAW = 1;
    /*DEBUG_PORT.println(micros());
    can100Hz();
    DEBUG_PORT.println(micros());
    can20Hz();
    DEBUG_PORT.println(micros());
    can10Hz();
    DEBUG_PORT.println(micros());*/
  } else if (ecuNumber==2) {
    // Setup Serial port
    DEBUG_PORT.begin(500000);
    // Setup Nextion and reset
    NEXTION_PORT.begin(250000);
    NEXTION_PORT.print(F("sleep=0")); //Wake Up Nextion
    endNextion();
    delay(500);
    NEXTION_PORT.print(F("dim=32"));  //Change brightness 0-100
    endNextion();
    NEXTION_PORT.print(F("page 0"));  //Page 0
    endNextion();
    NEXTION_PORT.print(F("thup=1"));  //allow wakeup from sleep
    endNextion();
    checkNext();
    // Setup CAN
    Can0.begin();
    Can0.setBaudRate(CANBUSSPEED0);
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
    my10Timer.begin (b10Set ,100000);  // blinkLED to run every 0.05 seconds 150000 = 0.15s
    my2Timer.begin   (b2Set ,500000);  // blinkLED to run every 0.50 seconds 150000 = 0.15s
    // Setup led Pin
    pinMode(ledDisp,OUTPUT);
    // Setup MCP23017s
    /*DEBUG_PORT.println(micros());
    can100Hz();
    DEBUG_PORT.println(micros());
    can20Hz();
    DEBUG_PORT.println(micros());
    can10Hz();
    DEBUG_PORT.println(micros());*/
  } else if (ecuNumber==3) {
    // Setup NEOPIXELS
    pixels.begin();
    pixels.setBrightness(127);
    // Setup SERIAL PORT
    DEBUG_PORT.begin(500000);
    // Setup ON CHIP CAN
    Can0.begin();
    Can0.setBaudRate(CANBUSSPEED0);
    Can0.setClock(CLK_60MHz);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(canSniff0);
    Can1.begin();
    Can1.setBaudRate(CANBUSSPEED1);
    Can1.setClock(CLK_60MHz);
    Can1.setMaxMB(16);
    Can1.enableFIFO();
    Can1.enableFIFOInterrupt();
    Can1.onReceive(canSniff1);
    Can2.begin();
    Can2.setBaudRate(CANBUSSPEED2);
    Can2.setClock(CLK_60MHz);
    Can2.setMaxMB(16);
    Can2.enableFIFO();
    Can2.enableFIFOInterrupt();
    Can2.onReceive(canSniff2);
    //Can0.mailboxStatus();
    //Can1.mailboxStatus();
    //Can2.mailboxStatus();
    // Setup MCP2515 CAN
    pinMode(CAN3_INT, INPUT);                     // Configuring pin for /INT input
  #if MCPADDPINS
    DEBUG_PORT.println("TXnBUF\t\tTRUE!");
    pixels.setPixelColor(0, pixels.Color(0,255,0));
  #else
    pixels.setPixelColor(0, pixels.Color(0,0,255));
  #endif
    pixels.show();
    if(CANMCP3.begin(MCP_ANY, CAN3_SPEED, MCP_8MHZ) == CAN_OK){
      DEBUG_PORT.print(F("CAN3:\t"));
      if (CAN3_SPEED == 12) {
        DEBUG_PORT.print(F("500kbps"));
      } else if (CAN3_SPEED == 9) {
        DEBUG_PORT.print(F("125kbps"));
      }
      DEBUG_PORT.print(F("\tInit OK!\r\n"));
      CANMCP3.setMode(MCP_NORMAL);
    } else {
      DEBUG_PORT.print(F("CAN3: Init Fail!!!\r\n"));
    }
    
    while (millis()<10000) {
      // do nothing
    }
  }
} // setup()

//**************************************************
// loop
//**************************************************

void loop() {
  if(ecuNumber==0) {
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
          BT_CAR_PORT.println(F("A#0#0#"));
          DEBUG_PORT.println(F("A#0#0#"));
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
        DEBUG_PORT.println(F("************************************************************"));
        DEBUG_PORT.print(F("mcpAValue            : "));
        if (mcpAValue < 0x10) DEBUG_PORT.print(F("0"));
        if (mcpAValue < 0x100) DEBUG_PORT.print(F("0"));
        if (mcpAValue < 0x1000) DEBUG_PORT.print(F("0"));
        DEBUG_PORT.println(mcpAValue,HEX);
        DEBUG_PORT.println(F("************************************************************"));
        DEBUG_PORT.print(F("Dial button          : "));
        DEBUG_PORT.println(dialButton);
        DEBUG_PORT.print(F("Dial           Dir   : "));
        DEBUG_PORT.print(curDialDir);
        DEBUG_PORT.print(F(" | counter: "));
        DEBUG_PORT.println(dialCount);
        DEBUG_PORT.print(F("potValue             : "));
        DEBUG_PORT.println(potValue,HEX);
        DEBUG_PORT.print(F("potValue_old         : "));
        DEBUG_PORT.println(potValue_old,HEX);
        DEBUG_PORT.print(F("engineValueRAW       : "));
        DEBUG_PORT.println(ecu_data.engineValueRAW,HEX);
        DEBUG_PORT.print(F("engineMalfunctionRAW : "));
        DEBUG_PORT.println(ecu_data.engineMalfunctionRAW,HEX);
        DEBUG_PORT.print(F("engineStatusRAW      : "));
        DEBUG_PORT.println(ecu_data.engineStatusRAW,HEX);
        DEBUG_PORT.print(F("speedKphRAW          : "));
        DEBUG_PORT.println(ecu_data.speedKphRAW,HEX);
        DEBUG_PORT.print(F("engineRpmRAW         : "));
        DEBUG_PORT.println(ecu_data.engineRpmRAW,HEX);
        DEBUG_PORT.print(F("fuelAmountRAW        : "));
        DEBUG_PORT.println(ecu_data.fuelAmountRAW,HEX);
        DEBUG_PORT.print(F("engineCoolantTempRAW : "));
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
  } else if (ecuNumber==1){
    Can0.events();
    checkCanValues();
    
    uint16_t mcpAValueRead = mcpA.readGPIOAB();
    uint16_t mcpAValue = ~mcpAValueRead;
    uint16_t mcpBValueRead = mcpB.readGPIOAB();
    uint16_t mcpBValue = ~mcpBValueRead;
    analog0.update();
    analog1.update();
    analog2.update();
    
    ecu_data.brakeValueRAW = analog0.getValue();
    ecu_data.acceleratorValueRAW = analog1.getValue();
    ecu_data.steeringValueRAW = map(analog2.getValue(),0,1023,-511,511);
    
    ecu_data.mcpA = mcpAValue;
    ecu_data.mcpB = mcpBValue;
  
    if((ecu_data.mcpA != ecu_data_old.mcpA) || (ecu_data.mcpB != ecu_data_old.mcpB) || (ecu_data.brakeValueRAW != ecu_data_old.brakeValueRAW) || (ecu_data.acceleratorValueRAW != ecu_data_old.acceleratorValueRAW) || (ecu_data.steeringValueRAW != ecu_data_old.steeringValueRAW)) {
      ecu_data.shiftValueRAW = ((mcpAValue & 0x0018) >> 3);
      
      if (((mcpAValue & 0x4000) >> 14) == 1) {
      ecu_data.engineValueRAW = !ecu_data.engineValueRAW;
      }
      
      ecu_data.turnSwitchValueRAW = ((mcpAValue & 0x1800) >> 11);
      if (((mcpAValue & 0x2000) >> 13) == 1) {
        ecu_data.hazardValueRAW = !ecu_data.hazardValueRAW;
      }
      if (ecu_data.hazardValueRAW) {
        ecu_data.turnSwitchValueRAW = (ecu_data.turnSwitchValueRAW | 4);
      }
      
      ecu_data.hornValueRAW = ((mcpAValue & 0x0080) >> 7); // was 0x0090 ??
      
      curLightBit0 = ((mcpAValue & 0x0001) >> 0);
      curLightBit1 = ((mcpAValue & 0x0002) >> 1);
      if ((curLightBit0 != preLightBit0) && (curLightBit0 == 1)) {
        if (curLightBit1 != curLightBit0) {
          lightCount++;
          curLightDir = " CW";
          if (lightCount > 2) {
            lightCount = 2;
          }
        } else {
          lightCount--;
          curLightDir = "CCW";
          if (lightCount < -1) {
            lightCount = -1;
          }
        }
      }
      if (lightCount != -1) {
        ecu_data.lightSwValueRAW = pow(2,lightCount);
      } else {
        ecu_data.lightSwValueRAW = 0;
      }
      preLightBit0 = curLightBit0;
      ecu_data.lightFlValueRAW = ((mcpAValue & 0x0004) >> 2);
      
      if (((mcpAValue & 0x8000) >> 15) == 1) {
      ecu_data.parkingValueRAW = !ecu_data.parkingValueRAW;
      }
      
      curWiperFrBit0 = ((mcpAValue & 0x0100) >> 8);
      curWiperFrBit1 = ((mcpAValue & 0x0200) >> 9);
      if ((curWiperFrBit0 != preWiperFrBit0) && (curWiperFrBit0 == 1)) {
        if (curWiperFrBit1 != curWiperFrBit0) {
          wiperFrCount++;
          curWiperFrDir = " CW";
          if (wiperFrCount > 2) {
            wiperFrCount = 2;
          }
        } else {
          wiperFrCount--;
          curWiperFrDir = "CCW";
          if (wiperFrCount < -1) {
            wiperFrCount = -1;
          }
        }
      }
      if (wiperFrCount != -1) {
        ecu_data.wiperFSwValueRAW = pow(2,wiperFrCount);
      } else {
        ecu_data.wiperFSwValueRAW = 0;
      }
      uint16_t wiperMist = (((mcpAValue & 0x0400) >> 10) << 3);
      ecu_data.wiperFSwValueRAW = (ecu_data.wiperFSwValueRAW | wiperMist);
      preWiperFrBit0 = curWiperFrBit0;
      
      ecu_data.wiperRSwValueRAW = (((mcpAValue & 0x0020) >> 5)|((mcpAValue & 0x0040) >> 3));
  
      boolean lLock = ((mcpBValue & 0x0004) >> 2);
      boolean rLock = ((mcpBValue & 0x0008) >> 3);
  
      if (lLock | rLock) {
        ecu_data.doorLockValueRAW = (ecu_data.doorLockValueRAW ^ lLock)^(rLock << 1);
      }
      
      ecu_data.lDoorSwValueRAW = ((mcpBValue & 0x0003) >> 0);
      
      ecu_data.rDoorSwValueRAW = ((mcpBValue & 0x0300) >> 8);
      if (gDebug) {
        DEBUG_PORT.println(F("************************************************************"));
        DEBUG_PORT.print(F("mcpAValue          : "));
        if (mcpAValue < 0x10) DEBUG_PORT.print(F("0"));
        if (mcpAValue < 0x100) DEBUG_PORT.print(F("0"));
        if (mcpAValue < 0x1000) DEBUG_PORT.print(F("0"));
        DEBUG_PORT.println(mcpAValue,HEX);
        DEBUG_PORT.print(F("mcpBValue          : "));
        if (mcpBValue < 0x10) DEBUG_PORT.print(F("0"));
        if (mcpBValue < 0x100) DEBUG_PORT.print(F("0"));
        if (mcpBValue < 0x1000) DEBUG_PORT.print(F("0"));
        DEBUG_PORT.println(mcpBValue,HEX);
        DEBUG_PORT.println(F("************************************************************"));
        DEBUG_PORT.print(F("brake              : "));
        if (ecu_data.brakeValueRAW < 10) DEBUG_PORT.print(F("0"));
        if (ecu_data.brakeValueRAW < 100) DEBUG_PORT.print(F("0"));
        if (ecu_data.brakeValueRAW < 1000) DEBUG_PORT.print(F("0"));
        DEBUG_PORT.println(ecu_data.brakeValueRAW);
        DEBUG_PORT.print(F("accelerator        : "));
        if (ecu_data.acceleratorValueRAW < 10) DEBUG_PORT.print(F("0"));
        if (ecu_data.acceleratorValueRAW < 100) DEBUG_PORT.print(F("0"));
        if (ecu_data.acceleratorValueRAW < 1000) DEBUG_PORT.print(F("0"));
        DEBUG_PORT.println(ecu_data.acceleratorValueRAW);
        DEBUG_PORT.print(F("steering           : "));
        if (ecu_data.steeringValueRAW > 0) {
          DEBUG_PORT.print(F("+"));
          if (ecu_data.steeringValueRAW < 10) DEBUG_PORT.print(F("0"));
          if (ecu_data.steeringValueRAW < 100) DEBUG_PORT.print(F("0"));
        } else {
          DEBUG_PORT.print(F("-"));
          if (ecu_data.steeringValueRAW > -10) DEBUG_PORT.print(F("0"));
          if (ecu_data.steeringValueRAW > -100) DEBUG_PORT.print(F("0"));
        }
        DEBUG_PORT.println(abs(ecu_data.steeringValueRAW));
        DEBUG_PORT.println(F("************************************************************"));
        DEBUG_PORT.print(F("shiftValueRAW        : "));
        DEBUG_PORT.println(ecu_data.shiftValueRAW,HEX);
        DEBUG_PORT.print(F("engineValueRAW       : "));
        DEBUG_PORT.println(ecu_data.engineValueRAW,HEX);
        DEBUG_PORT.print(F("engineStatusRAW      : "));
        DEBUG_PORT.println(ecu_data.engineStatusRAW,HEX);
        DEBUG_PORT.print(F("engineMalfunctionRAW : "));
        DEBUG_PORT.println(ecu_data.engineMalfunctionRAW,HEX);
        DEBUG_PORT.print(F("hazardValueRAW       : "));
        DEBUG_PORT.println(ecu_data.hazardValueRAW,HEX);
        DEBUG_PORT.print(F("turnSwitchValueRAW   : "));
        DEBUG_PORT.println(ecu_data.turnSwitchValueRAW,HEX);
        DEBUG_PORT.print(F("hornValueRAW         : "));
        DEBUG_PORT.println(ecu_data.hornValueRAW,HEX);
        DEBUG_PORT.print(F("Light            Dir : "));
        DEBUG_PORT.print(curLightDir);
        DEBUG_PORT.print(F(" | counter: "));
        DEBUG_PORT.println(lightCount);
        DEBUG_PORT.print(F("lightSwValueRAW      : "));
        DEBUG_PORT.println(ecu_data.lightSwValueRAW,HEX);
        DEBUG_PORT.print(F("lightFlValueRAW      : "));
        DEBUG_PORT.println(ecu_data.lightFlValueRAW,HEX);
        DEBUG_PORT.print(F("parkingValueRAW      : "));
        DEBUG_PORT.println(ecu_data.parkingValueRAW,HEX);
        DEBUG_PORT.print(F("FrWiper          Dir : "));
        DEBUG_PORT.print(curWiperFrDir);
        DEBUG_PORT.print(F(" | counter: "));
        DEBUG_PORT.println(wiperFrCount);
        DEBUG_PORT.print(F("wiperFSwValueRAW     : "));
        DEBUG_PORT.println(ecu_data.wiperFSwValueRAW,HEX);
        DEBUG_PORT.print(F("wiperRSwValueRAW     : "));
        DEBUG_PORT.println(ecu_data.wiperRSwValueRAW,HEX);
        DEBUG_PORT.print(F("doorLockValueRAW     : "));
        DEBUG_PORT.println(ecu_data.doorLockValueRAW,HEX);
        DEBUG_PORT.print(F("lDoorSwValueRAW      : "));
        DEBUG_PORT.println(ecu_data.lDoorSwValueRAW,HEX);
        DEBUG_PORT.print(F("rDoorSwValueRAW      : "));
        DEBUG_PORT.println(ecu_data.rDoorSwValueRAW,HEX);
        DEBUG_PORT.print(F("nextPage             : "));
        DEBUG_PORT.println(nextPage);
      }
      ecu_data_old.mcpA = mcpAValue;
      ecu_data_old.mcpB = mcpBValue;
    }
    
    dispNext();
    updateCanValues();
  
    if (b100Hz == 1) {
      can100Hz();
    }
    if (b20Hz == 1) {
      can20Hz();
    }
    if (b10Hz == 1) {
      can10Hz();
    }
  
    checkNext();
  } else if (ecuNumber==2){
    Can0.events();
    checkCanValues();
  
    /*unsigned long currentMillis = millis();
    if (currentMillis - prevMillis > interval) {
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
  
    ecu_data.turnSignalIndicatorRAW = ecu_data.turnSwitchValueRAW;
    if (((ecu_data.turnSignalIndicatorRAW & 0x04) >> 2) == 1) {
      ecu_data.turnSignalIndicatorRAW = 0x03;
    }
    ecu_data.frontWiperStatusRAW = ecu_data.wiperFSwValueRAW;
    ecu_data.rearWiperStatusRAW = ecu_data.wiperRSwValueRAW;
    ecu_data.lightIndicatorRAW = (ecu_data.lightSwValueRAW|(ecu_data.lightFlValueRAW<<2));
    ecu_data.doorLockStatusRAW = (ecu_data.doorLockStatusRAW & 0x0c)|(ecu_data.doorLockValueRAW & 0x03);
    
    dispNext();
    updateCanValues();
  
    if (b100Hz == 1) {
      can100Hz();
    }
    if (b20Hz == 1) {
      can20Hz();
    }
    if (b10Hz == 1) {
      can10Hz();
    }
    if (b2Hz == 1) {
      can2Hz();
    }
  
    checkNext();
  } else if(ecuNumber==3) {
    serialMenu();
    if ((millis()>21000)&(millis()<21005)&(proofDebug)) {
      DEBUG_PORT.print(F("CAN0:"));
      DEBUG_PORT.print(cnt0);
      DEBUG_PORT.print(F(" CAN1:"));
      DEBUG_PORT.print(cnt1);
      DEBUG_PORT.print(F(" CAN2:"));
      DEBUG_PORT.print(cnt2);
      DEBUG_PORT.print(F(" CAN3:0:"));
      DEBUG_PORT.print(cnt30);
      DEBUG_PORT.print(F(" 1:"));
      DEBUG_PORT.print(cnt31);
      DEBUG_PORT.print(F(" 2:"));
      DEBUG_PORT.print(cnt32);
      DEBUG_PORT.print(F(" Total:"));
      DEBUG_PORT.println(cnt30+cnt31+cnt32);
      //while(true){
        //
      //}
    }
    Can0.events();
    Can1.events();
    Can2.events();
    canSniff3();
    pixels.setPixelColor(0, pixels.Color(0,0,0));
    pixels.show();
  }
} // loop()
/**************************************************
 END FILE
**************************************************/

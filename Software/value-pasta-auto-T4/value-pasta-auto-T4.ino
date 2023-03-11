                                                  // Teensyduino 1.56
                                                  // Arduino 1.8.19
#include <SPI.h>                                  // version 1.0
#include <Wire.h>                                 // version 1.0
#include <FlexCAN_T4.h>                           // version 2018
#include <mcp_can.h>                              // version 1.5 25/09/17 from https://github.com/coryjfowler/MCP_CAN_lib modified for 10MHz SPI
#include "mcp_minty.h"
#include <Adafruit_MCP23X17.h>                    // version 2.1.0
//#include <Adafruit_BusIO.h>                     // version 1.11.6
#include <Adafruit_NeoPixel.h>                    // version 1.10.5
#include <ResponsiveAnalogRead.h>                 // version 1.2.1
#define             strVERSION  20230311          // date of upload

// 0 Powertrain
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;   // ALL: CAN0 Bus
// 1 Chassis
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;   // GW: CAN1 Bus
// 2 Body
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can2;   // GW: CAN2 Bus

// 0 Powertrain
const int CANBUSSPEED0  = 500000;                 // ALL: CAN0 speed
// 1 Chassis
const int CANBUSSPEED1  = 500000;                 // GW: CAN1 speed
// 2 Body
const int CANBUSSPEED2  = 500000;                 // GW: CAN2 speed
// 3 OBD2
const int CANBUSSPEED3  = 500000;                 // GW: CAN3 speed

#define             CAN3_INT    9                 // GW: Set INT to pin 9
#define             CAN3_CS     10                // GW: Set INT to pin 10
#define             CAN3_SPEED  CAN_500KBPS       // GW: 500kbps

#define             CAN3_TX0BUF 24                // GW: TX0 RTS Pin
//#define             CAN3_TX1BUF 25                // GW: TX1 RTS Pin
//#define             CAN3_TX2BUF 26                // GW: TX2 RTS Pin
//#define             CAN3_RX0BF  27                // GW: RX0 INT Pin
//#define             CAN3_RX1BF  28                // GW: RX1 INT Pin

MCP_CAN             CANMCP3(CAN3_CS);             // GW: CAN3 interface using CS on digital pin 10
MCP_CAN_MINTY       CANMCP3MINTY(CAN3_CS,CAN3_TX0BUF);
                                                  // GW: CAN3 library to use TX0BUF for sending

Adafruit_MCP23X17   mcpA;                         // PT: & CH: MCP23017
Adafruit_MCP23X17   mcpB;                         // CH:  MCP23017

#define             MAX_MSGS    32                // PT/CH/BO Used for triggered msgs
#define             MAX_PIXELS  1                 // GW: Number of Neopixels
#define             ECU_NUM_HI  2                 // ALL: Used to ID ECU
#define             ECU_NUM_LO  3                 // ALL: Used to ID ECU
#define             NEO_PIN     4                 // GW: Pin for Neopixel/LED

Adafruit_NeoPixel pixels(MAX_PIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);
                                                  // GW: Define Neopixel

const byte ana0x01A     = A0;                     // PT: CH: Analog0
const byte ana0x02F     = A1;                     // CH: Analog1
const byte ana0x058     = A2;                     // CH: Analog2
ResponsiveAnalogRead analog0(ana0x01A,true);      // PT: CH: Analog0
ResponsiveAnalogRead analog1(ana0x02F,true);      // CH: Analog1
ResponsiveAnalogRead analog2(ana0x058,true);      // CH: Analog2

IntervalTimer my100Timer;                         // PT: CH: BO: Define Timer
IntervalTimer my20Timer;                          // PT: CH: BO: Define Timer
IntervalTimer my10Timer;                          // CH: BO: Define Timer
IntervalTimer my2Timer;                           // PT: BO: Define Timer

#define     DEBUG_PORT    Serial                  // ALL: Debug Port
#define     BT_CAR_PORT   Serial4                 // PT: BT for Arduino CAR
#define     NEXTION_PORT  Serial5                 // PT: CH: BO: Nextion Display

boolean gDebug          = false;                  // ALL: show Debug

static CAN_message_t out_msg;                     // ALL: Used for outgoing msgs

byte ecuNumber          = 0;                      // NUMBER IS READ FROM D2 & D3:0=PT,1=CH,2=BO,3=GW

byte nextPage           = 0;                      // PT: CH: BO: Nextion Current Page
const byte msgSpacing   = 200;                    // PT: CH: BO: Used for CAN propagation

boolean b100Hz          = 0;                      // PT: CH: BO: Timer trigger
boolean b20Hz           = 0;                      // PT: CH: BO: Timer trigger
boolean b10Hz           = 0;                      // CH: BO: Timer trigger
boolean b2Hz            = 0;                      // PT: BO: Timer trigger

uint16_t potValue       = 0;                      // PT: Used for Potentiometer
uint16_t potValue_old   = 0;                      // PT: Used for Potentiometer

boolean curDialBit0     = 0;                      // PT: Used for Rotary Encoder
boolean curDialBit1     = 0;                      // PT: Used for Rotary Encoder
boolean preDialBit0     = 0;                      // PT: Used for Rotary Encoder
boolean preDialBit1     = 0;                      // PT: Used for Rotary Encoder
int dialCount           = 0;                      // PT: Used for Rotary Encoder
boolean dialButton      = 0;                      // PT: Used for Rotary Encoder
String curDialDir       = "   ";                  // PT: Used for Rotary Encoder

boolean curLightBit0    = 0;                      // CH: Used for Light Rotary Encoder
boolean curLightBit1    = 0;                      // CH: Used for Light Rotary Encoder
boolean preLightBit0    = 0;                      // CH: Used for Light Rotary Encoder
boolean preLightBit1    = 0;                      // CH: Used for Light Rotary Encoder
int lightCount          = -1;                     // CH: Used for Light Rotary Encoder
String curLightDir      = "   ";                  // CH: Used for Light Rotary Encoder

boolean curWiperFrBit0  = 0;                      // CH: Used for Wiper Rotary Encoder
boolean curWiperFrBit1  = 0;                      // CH: Used for Wiper Rotary Encoder
boolean preWiperFrBit0  = 0;                      // CH: Used for Wiper Rotary Encoder
boolean preWiperFrBit1  = 0;                      // CH: Used for Wiper Rotary Encoder
int wiperFrCount        = -1;                     // CH: Used for Wiper Rotary Encoder
String curWiperFrDir    = "   ";                  // CH: Used for Wiper Rotary Encoder

unsigned long lastCanMsg= 0;                      // PT: CH: BO: Used to check last msg time

boolean proofDebug      = true;                   // GW: to count msgs/s
boolean firewallOpen0   = false;                  // GW: firewall for PT
boolean firewallOpen1   = false;                  // GW: firewall for CH
boolean firewallOpen2   = false;                  // GW: firewall for BO

const unsigned long unlockId = 0x123;             // GW: CAN-ID to unlock the ODB2 firewall
const unsigned long lockId   = 0x124;             // GW: CAN-ID to lock the ODB2 firewall
byte unlockBuf[8]            = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
                                                  // GW: data to lock/unlock byte 0 configures which ECU

long unsigned int rxId;                           // GW: Used for MCP3 received msgs
unsigned char len       = 0;                      // GW: Used for MCP3 received msgs
unsigned char rxBuf[8];                           // GW: Used for MCP3 received msgs

unsigned long cntStart  = 20000;                  // GW: Used to count msg/s
unsigned int cnt0       = 0;                      // GW: Used to count msg/s
unsigned int cnt1       = 0;                      // GW: Used to count msg/s
unsigned int cnt2       = 0;                      // GW: Used to count msg/s
unsigned int cnt30      = 0;                      // GW: Used to count msg/s
unsigned int cnt31      = 0;                      // GW: Used to count msg/s
unsigned int cnt32      = 0;                      // GW: Used to count msg/s

unsigned long debounceDelay           = 50;       // CH: Used to debounce push switches
unsigned long lastEngineDebounceTime  = 0;        // CH: Used to debounce push switches
unsigned long lastPBrakeDebounceTime  = 0;        // CH: Used to debounce push switches
unsigned long lastHazardDebounceTime  = 0;        // CH: Used to debounce push switches
unsigned long lastHornDebounceTime    = 0;        // CH: Used to debounce push switches
unsigned long lastLockDebounceTime    = 0;        // CH: Used to debounce push switches

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

struct MSG_DEF {
  boolean rtr;
  unsigned int id;
  unsigned int dlc;
  boolean enb;
  boolean rep;
  unsigned int freq;
  unsigned int cnt;
} msg_data;

MSG_DEF msg_array[32];

//**************************************************
// define_rep_msg()
//**************************************************

void define_rep_msg (boolean rtr, unsigned int id, unsigned int dlc, boolean enb, boolean rep, unsigned int freq, unsigned int cnt){
  for (int i=0; i<MAX_MSGS; i++) {
    if (msg_array[i].id==0) {
      msg_array[i].rtr = rtr;
      msg_array[i].id = id;
      msg_array[i].dlc = dlc;
      msg_array[i].enb = enb;
      msg_array[i].rep = rep;
      msg_array[i].freq = freq;
      msg_array[i].cnt = cnt;
      break;
    }
  }
} // define_rep_msg()

//**************************************************
// b100Set()
//**************************************************

void b100Set() {
  b100Hz = true;
} // b100Set()

//**************************************************
// b20Set()
//**************************************************

void b20Set() {
  b20Hz = true;
} // b20Set()

//**************************************************
// b10Set()
//**************************************************

void b10Set() {
  b10Hz = true;
} // b10Set()

//**************************************************
// b2Set()
//**************************************************

void b2Set() {
  b2Hz = true;
} // b2Set()

//**************************************************
// can100Hz()
//**************************************************

void can100Hz() {
  for (int i=0; i<MAX_MSGS; i++) {
    if ((msg_array[i].enb) & (msg_array[i].freq==10)) {
      out_msg.len = msg_array[i].dlc;
      out_msg.id = msg_array[i].id;
      switch (msg_array[i].id) {
        case brakeOutputIndMSG:
          out_msg.buf[0] = ((ecu_data.brakeValueRAW >> 8) & 0xFF);
          out_msg.buf[1] = ((ecu_data.brakeValueRAW >> 0) & 0xFF);
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case throttlePositionMSG:
          out_msg.buf[0] = ((ecu_data.acceleratorValueRAW >> 8) & 0xFF);
          out_msg.buf[1] = ((ecu_data.acceleratorValueRAW >> 0) & 0xFF);
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case engineRpmMSG:
          out_msg.buf[0] = ((ecu_data.engineRpmRAW >> 8) & 0xFF);
          out_msg.buf[1] = ((ecu_data.engineRpmRAW >> 0) & 0xFF);
          out_msg.buf[2] = ((ecu_data.speedKphRAW >> 8) & 0xFF);
          out_msg.buf[3] = ((ecu_data.speedKphRAW >> 0) & 0xFF);
          Can0.write(out_msg);
          break;
        case powerSteeringOutIndMSG:
          out_msg.buf[0] = ((ecu_data.steeringValueRAW >> 8) & 0xFF);
          out_msg.buf[1] = ((ecu_data.steeringValueRAW >> 0) & 0xFF);
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case shiftPositionMSG:
          out_msg.buf[0] = ecu_data.shiftPositionRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case brakeOperationMSG:
          out_msg.buf[0] = ((ecu_data.brakeValueRAW >> 8) & 0xFF);
          out_msg.buf[1] = ((ecu_data.brakeValueRAW >> 0) & 0xFF);
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case accelerationOperationMSG:
          out_msg.buf[0] = ((ecu_data.acceleratorValueRAW >> 8) & 0xFF);
          out_msg.buf[1] = ((ecu_data.acceleratorValueRAW >> 0) & 0xFF);
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case steeringWheelPosMSG:
          out_msg.buf[0] = ((ecu_data.steeringValueRAW >> 8) & 0xFF);
          out_msg.buf[1] = ((ecu_data.steeringValueRAW >> 0) & 0xFF);
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case shiftPositionSwitchMSG:
          out_msg.buf[0] = ecu_data.shiftValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case engineStartMSG:
          out_msg.buf[0] = ecu_data.engineValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case turnSwitchMSG:
          out_msg.buf[0] = ecu_data.turnSwitchValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case hornSwitchMSG:
          out_msg.buf[0] = ecu_data.hornValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case turnSignalIndicatorMSG:
          out_msg.buf[0] = ecu_data.turnSignalIndicatorRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case hornOperationMSG:
          out_msg.buf[0] = ecu_data.hornValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case airbagActivationMSG:
          out_msg.buf[0] = 0x0;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        default:
          break;
      }
      delayMicroseconds(msgSpacing);
    } else if (msg_array[i].freq==0) {
      break;
    }
  }
  b100Hz = false;
  digitalWrite(NEO_PIN,!digitalRead(NEO_PIN));
} // can100Hz()

//**************************************************
// can20Hz()
//**************************************************

void can20Hz() {
  for (int i=0; i<MAX_MSGS; i++) {
    if ((msg_array[i].enb) & (msg_array[i].freq==50)) {
      out_msg.len = msg_array[i].dlc;
      out_msg.id = msg_array[i].id;
      switch (msg_array[i].id) {
        case brakeOilIndMSG:
          out_msg.buf[0] = 0xff;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case absBrakeOperationMSG:
          out_msg.buf[0] = 0x0;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case throttleAdjustmentMSG:
          out_msg.buf[0] = 0x0;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case engineCoolantTempMSG:
          out_msg.buf[0] = ecu_data.engineCoolantTempRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case engineMalfunctionMSG:
          out_msg.buf[0] = ecu_data.engineMalfunctionRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case powerSteeringMalfMSG:
          out_msg.buf[0] = 0x0;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case engineStatusMSG:
          out_msg.buf[0] = ecu_data.engineStatusRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case parkingBrakeStatusMSG:
          out_msg.buf[0] = ecu_data.parkingBrakeStatusRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case lightSwitchMSG:
          out_msg.buf[0] = ecu_data.lightSwValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case lightFlashMSG:
          out_msg.buf[0] = ecu_data.lightFlValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case parkingBrakeMSG:
          out_msg.buf[0] = ecu_data.parkingValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case lightIndicatorMSG:
          out_msg.buf[0] = ecu_data.lightIndicatorRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        default:
          break;
      }
      delayMicroseconds(msgSpacing);
    } else if (msg_array[i].freq==0) {
      break;
    }
  }
  b20Hz = false;
  digitalWrite(NEO_PIN,!digitalRead(NEO_PIN));
} // can20Hz()

//**************************************************
// can10Hz()
//**************************************************

void can10Hz() {
  for (int i=0; i<MAX_MSGS; i++) {
    if ((msg_array[i].enb) & (msg_array[i].freq==100)) {
      out_msg.len = msg_array[i].dlc;
      out_msg.id = msg_array[i].id;
      switch (msg_array[i].id) {
        case wiperSwitchFrontMSG:
          out_msg.buf[0] = ecu_data.wiperFSwValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case wiperSwitchRearMSG:
          out_msg.buf[0] = ecu_data.wiperRSwValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case doorLockUnlockMSG:
          out_msg.buf[0] = ecu_data.doorLockValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case lDoorSwWindowMSG:
          out_msg.buf[0] = ecu_data.lDoorSwValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case rDoorSwWindowMSG:
          out_msg.buf[0] = ecu_data.rDoorSwValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case frontWiperStatusMSG:
          out_msg.buf[0] = ecu_data.wiperFSwValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case rearWiperStatusMSG:
          out_msg.buf[0] = ecu_data.wiperRSwValueRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case doorLockStatusMSG:
          out_msg.buf[0] = ecu_data.doorLockStatusRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case lDoorPositionMSG:
          if (ecu_data.lDoorLimitRAW==0) {
            out_msg.buf[0] = 0x0;
            out_msg.buf[1] = 0x01;
            out_msg.buf[2] = 0x0;
            out_msg.buf[3] = 0x0;
          } else if (ecu_data.lDoorLimitRAW==100) {
            out_msg.buf[0] = 0x64;
            out_msg.buf[1] = 0x02;
            out_msg.buf[2] = 0x0;
            out_msg.buf[3] = 0x0;
          } else {
            out_msg.buf[0] = ecu_data.lDoorLimitRAW;
            out_msg.buf[1] = 0x0;
            out_msg.buf[2] = 0x0;
            out_msg.buf[3] = 0x0;
          }
          Can0.write(out_msg);
          break;
        case rDoorPositionMSG:
          if (ecu_data.rDoorLimitRAW==0) {
            out_msg.buf[0] = 0x0;
            out_msg.buf[1] = 0x01;
            out_msg.buf[2] = 0x0;
            out_msg.buf[3] = 0x0;
          } else if (ecu_data.rDoorLimitRAW==100) {
            out_msg.buf[0] = 0x64;
            out_msg.buf[1] = 0x02;
            out_msg.buf[2] = 0x0;
            out_msg.buf[3] = 0x0;
          } else {
            out_msg.buf[0] = ecu_data.rDoorLimitRAW;
            out_msg.buf[1] = 0x0;
            out_msg.buf[2] = 0x0;
            out_msg.buf[3] = 0x0;
          }
          Can0.write(out_msg);
          break;
        default:
          break;
      }
      delayMicroseconds(msgSpacing);
    } else if (msg_array[i].freq==0) {
      break;
    }
  }
  b10Hz = false;
  digitalWrite(NEO_PIN,!digitalRead(NEO_PIN));
} // can10Hz()

//**************************************************
// can2Hz()
//**************************************************

void can2Hz() {
  for (int i=0; i<MAX_MSGS; i++) {
    if ((msg_array[i].enb) & (msg_array[i].freq==500)) {
      out_msg.len = msg_array[i].dlc;
      out_msg.id = msg_array[i].id;
      switch (msg_array[i].id) {
        case fuelAmountMSG:
          out_msg.buf[0] = ecu_data.fuelAmountRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case batteryWarningMSG:
          out_msg.buf[0] = !ecu_data.engineStatusRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case ecoDrivingJudgementMSG:
          out_msg.buf[0] = 0x0;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case doorDriveUnitMalfuncMSG:
          out_msg.buf[0] = 0x0;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case seatBeltSensorMSG:
          out_msg.buf[0] = 0x0;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case seatBeltAlarmMSG:
          out_msg.buf[0] = 0x0;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case bonnetOpenSwitchMSG:
          out_msg.buf[0] = ecu_data.bonnetOpenSwitchRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        case trunkOpenSwitchMSG:
          out_msg.buf[0] = ecu_data.trunkOpenSwitchRAW;
          out_msg.buf[1] = 0x0;
          out_msg.buf[2] = 0x0;
          out_msg.buf[3] = 0x0;
          Can0.write(out_msg);
          break;
        default:
          break;
      }
      delayMicroseconds(msgSpacing);
    } else if (msg_array[i].freq==0) {
      break;
    }
  }
  b2Hz = false;
  digitalWrite(NEO_PIN,!digitalRead(NEO_PIN));
} // can2Hz()

//**************************************************
// canSniff0()
//**************************************************

void canSniff0(const CAN_message_t &msg0) {
  if ((gDebug)&(ecuNumber==3)) DEBUG_PORT.println(F("RX CAN0"));
  int msgID = msg0.id;
  switch (msgID) {
    //100Hz PT/BO
    case brakeOperationMSG:
      ecu_data.brakeValueRAW = ((msg0.buf[0] << 8) | (msg0.buf[1]));
      break;
    //100Hz PT
    case accelerationOperationMSG:
      ecu_data.acceleratorValueRAW = ((msg0.buf[0] << 8) | (msg0.buf[1]));
      break;
    //100Hz PT
    case steeringWheelPosMSG:
      ecu_data.steeringValueRAW = ((msg0.buf[0] << 8) | (msg0.buf[1]));
      break;
    //100Hz PT
    case shiftPositionSwitchMSG:
      ecu_data.shiftValueRAW = (msg0.buf[0]);
      break;
    //100Hz PT
    case engineStartMSG:
      ecu_data.engineValueRAW = (msg0.buf[0]);
      break;
    //100Hz PT/BO
    case turnSwitchMSG:
      ecu_data.turnSwitchValueRAW = (msg0.buf[0] & 0x07);
      break;
    //100Hz PT/BO
    case hornSwitchMSG:
      ecu_data.hornValueRAW = (msg0.buf[0] & 0x01);
      break;
    //100Hz CH
    case brakeOutputIndMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.brakeOutputRAW = ((msg0.buf[0] << 8) | (msg0.buf[1]));
      }
      break;
    //100Hz CH
    case throttlePositionMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.throttlePositionRAW = ((msg0.buf[0] << 8) | (msg0.buf[1]));
      }
      break;
    //100Hz CH
    case engineRpmMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.engineRpmRAW = ((msg0.buf[0] << 8) | (msg0.buf[1]));
        ecu_data.speedKphRAW = ((msg0.buf[2] << 8) | (msg0.buf[3]));
      }
      break;
    //100Hz CH
    case powerSteeringOutIndMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.powerSteeringRAW = ((msg0.buf[0] << 8) | (msg0.buf[1]));
        ecu_data.powerSteeringTorqueRAW = ((msg0.buf[2] << 8) | (msg0.buf[3]));
      }
      break;
    //100Hz CH
    case shiftPositionMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.shiftPositionRAW = (msg0.buf[0]);
        switch (ecu_data.shiftPositionRAW) {
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
      }
    //100Hz CH
    case turnSignalIndicatorMSG:
      ecu_data.turnSignalIndicatorRAW = (msg0.buf[0]);
      break;
    //100Hz CH
    case hornOperationMSG:
      ecu_data.hornOperationRAW = (msg0.buf[0]);
      mcpB.digitalWrite(7,ecu_data.hornOperationRAW & 0x01);
      break;
    //100Hz CH
    case airbagActivationMSG:
      ecu_data.airbagActivationRAW = (msg0.buf[0]);
      break;
    //20Hz PT/BO
    case lightSwitchMSG:
      ecu_data.lightSwValueRAW = (msg0.buf[0] & 0x07);
      break;
    //20Hz PT/BO
    case lightFlashMSG:
      ecu_data.lightFlValueRAW = (msg0.buf[0] & 0x01);
      break;
    //20Hz PT
    case parkingBrakeMSG:
      ecu_data.parkingValueRAW = (msg0.buf[0]);
      break;
    //20Hz CH
    case brakeOilIndMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.brakeOilIndRAW = (msg0.buf[0]);
      }
      break;
    //20Hz CH
    case absBrakeOperationMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.absBrakeOperationRAW = (msg0.buf[0]);
      }
      break;
    //20Hz CH
    case throttleAdjustmentMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.throttleAdjustmentRAW = (msg0.buf[0]);
      }
      break;
    //20Hz CH
    case engineCoolantTempMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.engineCoolantTempRAW = (msg0.buf[0]);
        ecu_data.engineCoolantTempNum = ecu_data.engineCoolantTempRAW - 40;
      }
      break;
    //20Hz CH
    case engineMalfunctionMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.engineMalfunctionRAW = (msg0.buf[0]);
      }
      break;
    //20Hz CH
    case powerSteeringMalfMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.powerSteeringMalfRAW = (msg0.buf[0]);
      }
      break;
    //20Hz CH
    case engineStatusMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.engineStatusRAW = (msg0.buf[0] & 0x01);
        mcpB.digitalWrite(14,ecu_data.engineStatusRAW);
      }
      break;
    //20Hz CH
    case parkingBrakeStatusMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.parkingBrakeStatusRAW = (msg0.buf[0] & 0x01);
        mcpB.digitalWrite(15,ecu_data.parkingBrakeStatusRAW);
      }
      break;
    //20Hz CH
    case lightIndicatorMSG:
      ecu_data.lightIndicatorRAW = (msg0.buf[0]);
      break;
    //10Hz CH
    case frontWiperStatusMSG:
      ecu_data.frontWiperStatusRAW = ((msg0.buf[1] & 0x0f));
      ecu_data.frontWiperTimerRAW = ((((msg0.buf[1] << 8) | (msg0.buf[0])) & 0xfff0) >> 7);
      break;
    //10Hz CH
    case rearWiperStatusMSG:
      ecu_data.rearWiperStatusRAW = (msg0.buf[0]);
      break;
    //10Hz CH
    case doorLockStatusMSG:
      ecu_data.doorLockStatusRAW = (msg0.buf[0]);
      if (((ecu_data.doorLockStatusRAW & 0x04) >> 2) & ((ecu_data_old.doorLockStatusRAW & 0x01) >> 0)) {
        ecu_data.doorLockValueRAW = ecu_data.doorLockValueRAW & 0xfe;
      } else if (((ecu_data.doorLockStatusRAW & 0x08) >> 3) & ((ecu_data_old.doorLockStatusRAW & 0x02) >> 1)) {
        ecu_data.doorLockValueRAW = ecu_data.doorLockValueRAW & 0xfd;
      }
      break;
    //10Hz CH
    case lDoorPositionMSG:
      ecu_data.lDoorPositionRAW = (msg0.buf[0]);
      ecu_data.lDoorLimitRAW = (msg0.buf[1]);
      break;
    //10Hz CH
    case rDoorPositionMSG:
      ecu_data.rDoorPositionRAW = (msg0.buf[0]);
      ecu_data.rDoorLimitRAW = (msg0.buf[1]);
      break;
    //10Hz BO
    case wiperSwitchFrontMSG:
      ecu_data.wiperFSwValueRAW = (msg0.buf[0]);
      break;
    //10Hz BO
    case wiperSwitchRearMSG:
      ecu_data.wiperRSwValueRAW = (msg0.buf[0]);
      break;
    //10Hz BO
    case doorLockUnlockMSG:
      ecu_data.doorLockValueRAW = (msg0.buf[0]);
      break;
    //10Hz BO
    case lDoorSwWindowMSG:
      ecu_data.lDoorSwValueRAW = (msg0.buf[0]);
      break;
    //10Hz BO
    case rDoorSwWindowMSG:
      ecu_data.rDoorSwValueRAW = (msg0.buf[0]);
      break;
    //2Hz CH
    case fuelAmountMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.fuelAmountRAW = (msg0.buf[0]);
      }
      break;
    //2Hz CH
    case batteryWarningMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.batteryWarningRAW = (msg0.buf[0]);
      }
      break;
    //2Hz CH
    case ecoDrivingJudgementMSG:
      if (ecuNumber==3) {
        // 1
        Can1.write(msg0);
      } else {
        ecu_data.ecoDrivingJudgementRAW = (msg0.buf[0]);
      }
      break;
    //2Hz CH
    case doorDriveUnitMalfuncMSG:
      ecu_data.doorDriveUnitMalfuncRAW = (msg0.buf[0]);
      break;
    //2Hz CH
    case seatBeltSensorMSG:
      ecu_data.seatBeltSensorRAW = (msg0.buf[0]);
      break;
    //2Hz CH
    case seatBeltAlarmMSG:
      ecu_data.seatBeltAlarmRAW = (msg0.buf[0]);
      break;
    //2Hz CH
    case bonnetOpenSwitchMSG:
      ecu_data.bonnetOpenSwitchRAW = (msg0.buf[0]);
      break;
    //2Hz CH
    case trunkOpenSwitchMSG:
      ecu_data.trunkOpenSwitchRAW = (msg0.buf[0]);
      break;
    //RESET MSG PT/CH/BO
    case resetMSG:
      SCB_AIRCR = 0x05FA0004;
      break;
    default:
      break;
  }
  if (ecuNumber==3) {
    unsigned long millis0 = millis();
    if ((millis0>cntStart)&(millis0<(cntStart+1001))&(proofDebug)) {
      cnt0++;
    }
    if (firewallOpen0) {
      if (gDebug) {
        DEBUG_PORT.println(F("CAN0 to CAN3"));
      }
      //msg0.buf[5]=0xff;
      //byte sndStat = CANMCP3.sendMsgBuf(msg0.id,0,msg0.len,msg0.buf);
      byte sndStat = CANMCP3MINTY.sendTX0(msg0.id,msg0.len,msg0.buf,1);
      if ((millis0>cntStart)&(millis0<(cntStart+1001))&(proofDebug)&(sndStat==CAN_OK)) {
        cnt30++;
      }
    }
    pixels.setPixelColor(0, pixels.Color(150,0,0));
    pixels.show();
  }  
  lastCanMsg = millis();
} // canSniff0()

//**************************************************
// canSniff1()
//**************************************************

void canSniff1(const CAN_message_t &msg1) {
  if (gDebug) DEBUG_PORT.println(F("RX CAN1"));
  int msgID = msg1.id;
  switch (msgID) {
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
  unsigned long millis1 = millis();
  if ((millis1>cntStart)&(millis1<(cntStart+1001))&(proofDebug)) {
    cnt1++;
  }
  if (firewallOpen1) {
    if (gDebug) {
      DEBUG_PORT.println(F("CAN1 to CAN3"));
    }
    //msg1.buf[6]=0xff;
    //byte sndStat = CANMCP3.sendMsgBuf(msg1.id,0,msg1.len,msg1.buf);
    byte sndStat = CANMCP3MINTY.sendTX0(msg1.id,msg1.len,msg1.buf,1);
    if ((millis1>cntStart)&(millis1<(cntStart+1001))&(proofDebug)&(sndStat==CAN_OK)) {
      cnt31++;
    }
  }
  pixels.setPixelColor(0, pixels.Color(0,150,0));
  pixels.show();
} // canSniff1()

//**************************************************
// canSniff2()
//**************************************************

void canSniff2(const CAN_message_t &msg2) {
  if (gDebug) DEBUG_PORT.println(F("RX CAN2"));
  int msgID = msg2.id;
  switch (msgID) {
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
  unsigned long millis2 = millis();
  if ((millis2>cntStart)&(millis2<(cntStart+1001))&(proofDebug)) {
    cnt2++;
  }
  if (firewallOpen2) {
    if (gDebug) {
      DEBUG_PORT.println(F("CAN2 to CAN3"));
    }
    //msg2.buf[7]=0xff;
    //byte sndStat = CANMCP3.sendMsgBuf(msg2.id,0,msg2.len,msg2.buf);
    byte sndStat = CANMCP3MINTY.sendTX0(msg2.id,msg2.len,msg2.buf,1);
    if ((millis2>cntStart)&(millis2<(cntStart+1001))&(proofDebug)&(sndStat==CAN_OK)) {
      cnt32++;
    }
  }
  pixels.setPixelColor(0, pixels.Color(0,0,150));
  pixels.show();
} // canSniff2()

//**************************************************
// canSniff3()
//**************************************************

void canSniff3() {
  int checkno = 0;
  CAN_message_t msg3;
  if (!digitalRead(CAN3_INT)) {
    CANMCP3.readMsgBuf(&rxId, &len, rxBuf);
    checkno = rxBuf[0];
    switch (rxId) {
      case unlockId:
        if ((rxBuf[1] == unlockBuf[1])&(rxBuf[2] == unlockBuf[2])&(rxBuf[3] == unlockBuf[3])&(rxBuf[4] == unlockBuf[4])&(rxBuf[5] == unlockBuf[5])&(rxBuf[6] == unlockBuf[6])&(rxBuf[7] == unlockBuf[7])) {
          if ((checkno&0x01)>>0) {
            firewallOpen0 = true;
            DEBUG_PORT.println(F("Firewall 0\tOpen"));
          }
          if ((checkno&0x02)>>1) {
            firewallOpen1 = true;
            DEBUG_PORT.println(F("Firewall 1\tOpen"));
          }
          if ((checkno&0x04)>>2) {
            firewallOpen2 = true;
            DEBUG_PORT.println(F("Firewall 2\tOpen"));
          }
        }
        break;
      case lockId:
        if ((rxBuf[1] == unlockBuf[1])&(rxBuf[2] == unlockBuf[2])&(rxBuf[3] == unlockBuf[3])&(rxBuf[4] == unlockBuf[4])&(rxBuf[5] == unlockBuf[5])&(rxBuf[6] == unlockBuf[6])&(rxBuf[7] == unlockBuf[7])) {
          if ((checkno&0x01)>>0) {
            firewallOpen0 = false;
            DEBUG_PORT.println(F("Firewall 0\tClosed"));         
          }
          if ((checkno&0x02)>>1) {
            firewallOpen1 = false;
            DEBUG_PORT.println(F("Firewall 1\tClosed"));         
          }
          if ((checkno&0x04)>>2) {
            firewallOpen2 = false;
            DEBUG_PORT.println(F("Firewall 2\tClosed"));         
          }
        }
        break;
      case resetMSG:
        if ((checkno&0x01)>>0) {
          msg3.id = resetMSG;
          msg3.len = 0;
          Can0.write(msg3);
        }
        if ((checkno&0x02)>>1) {
          msg3.id = resetMSG;
          msg3.len = 0;
          Can1.write(msg3);
        }
        if ((checkno&0x04)>>2) {
          msg3.id = resetMSG;
          msg3.len = 0;
          Can2.write(msg3);
        }
        if ((checkno&0x08)>>3) {
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
  CAN_message_t reset_msg;
  reset_msg.id = resetMSG;
  reset_msg.len = 0x0;
  if (DEBUG_PORT.available()) {
    for (int i=0; i<8; i++) {
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
          case 'c':     // restart msg counter
            DEBUG_PORT.println(F("Restarting msg counter!"));
            DEBUG_PORT.println(F("Press C to see result !"));
            cnt0 = 0;
            cnt1 = 0;
            cnt2 = 0;
            cnt30 = 0;
            cnt31 = 0;
            cnt32 = 0;
            cntStart = millis();
            break;
          case 'C':     // show counter results
            DEBUG_PORT.print(F("Millis START: "));
            DEBUG_PORT.println(cntStart);
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
            break;
          case 'd':     // toggle DEBUG
            gDebug=!gDebug;
            break;
          case 'h':     // help
            DEBUG_PORT.print(F("**************************************************\r\n"));
            DEBUG_PORT.println(F("\tHELP FUNCTIONS"));
            DEBUG_PORT.print(F("**************************************************\r\n"));
            DEBUG_PORT.println();
            if (ecuNumber==3) {
              DEBUG_PORT.println(F("c\tRestart msg counter"));
              DEBUG_PORT.println(F("C\tSee counter result"));
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
            } else {
              DEBUG_PORT.println(F("r\tReboot ECU"));
            }
            DEBUG_PORT.println(F("d\tToggle Debug"));           
            DEBUG_PORT.println(F("u\tCurrent Millis()"));           
            break;
          case 'r':     // reboot ECUs
            if (ecuNumber<3) {
              delayMicroseconds(250);
              SCB_AIRCR = 0x05FA0004;
            } else if (cmdbuf[1] == '\r') {
              Can0.write(reset_msg);
              delayMicroseconds(250);
              Can1.write(reset_msg);
              delayMicroseconds(250);
              Can2.write(reset_msg);
              delayMicroseconds(250);
              SCB_AIRCR = 0x05FA0004;
            } else {
              if ((checkno&0x1)>>0) {
                Can0.write(reset_msg);
              }
              if ((checkno&0x2)>>1) {
                Can1.write(reset_msg);
              }
              if ((checkno&0x4)>>2) {
                Can2.write(reset_msg);
              }
              if ((checkno&0x8)>>3) {
                delayMicroseconds(250);
                SCB_AIRCR = 0x05FA0004;
              }
            }
            break;
          case 'f':     // firewall toggle
            if (cmdbuf[1] == '\r') {
              firewallOpen0 = !firewallOpen0;
              firewallOpen1 = !firewallOpen1;
              firewallOpen2 = !firewallOpen2;
            } else {
              if ((checkno & 0x01)>>0) {
                firewallOpen0 = !firewallOpen0;
              }
              if ((checkno & 0x02)>>1) {
                firewallOpen1 = !firewallOpen1;
              }
              if ((checkno & 0x04)>>2) {
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
          case 'u':     // show millis
            DEBUG_PORT.print(F("Millis: "));
            DEBUG_PORT.println(millis());
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
} // updateCanValues()

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
      DEBUG_PORT.println(F("D#2000#"));
    } else {
      BT_CAR_PORT.println(F("D#0#"));
      DEBUG_PORT.println(F("D#0#"));
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
} // btSend()

//**************************************************
// powertrainECUdata()
//**************************************************

void powertrainECUdata() {
  uint16_t mcpAValueRead = mcpA.readGPIOAB();
  uint16_t mcpAValue = ~mcpAValueRead;
  analog0.update();
  
  potValue = analog0.getValue();
  
  ecu_data.mcpA = mcpAValue;
  
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
      DEBUG_PORT.printf("%04x",mcpAValue);
      DEBUG_PORT.println();
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
  if ((ecu_data.shiftPositionRAW == 1) | (ecu_data.shiftPositionRAW == 3)) {
    if (dialCount > 5) {
      ecu_data.speedKphRAW = 0;
      ecu_data.engineRpmRAW = (ecu_data.acceleratorValueRAW * 8124 / 1024);
    }
  }
  dispNext();
  ecu_data_old.mcpA = mcpAValue;
  potValue_old = potValue;
} // powertrainECUdata()

//**************************************************
// chassisECUdata()
//**************************************************

void chassisECUdata() {
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

  if ((ecu_data.mcpA != ecu_data_old.mcpA)
    || (ecu_data.mcpB != ecu_data_old.mcpB)
    || (ecu_data.brakeValueRAW != ecu_data_old.brakeValueRAW)
    || (ecu_data.acceleratorValueRAW != ecu_data_old.acceleratorValueRAW)
    || (ecu_data.steeringValueRAW != ecu_data_old.steeringValueRAW)) {
    ecu_data.shiftValueRAW = ((mcpAValue & 0x0018) >> 3);
    
    ecu_data.turnSwitchValueRAW = ((mcpAValue & 0x1800) >> 11);
    
    if(ecu_data.hazardValueRAW != ecu_data_old.hazardValueRAW) {
      lastHazardDebounceTime = millis();
    }
    if ((millis() - lastHazardDebounceTime) > debounceDelay) {
      if (((mcpAValue & 0x2000) >> 13) == 1) {
        ecu_data.hazardValueRAW = !ecu_data.hazardValueRAW;
      }
      if (ecu_data.hazardValueRAW) {
        ecu_data.turnSwitchValueRAW = (ecu_data.turnSwitchValueRAW | 4);
      }
    }
    
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

    ecu_data.lDoorSwValueRAW = ((mcpBValue & 0x0003) >> 0);
    
    ecu_data.rDoorSwValueRAW = ((mcpBValue & 0x0300) >> 8);

    if(ecu_data.engineValueRAW != ecu_data_old.engineValueRAW) {
      lastEngineDebounceTime = millis();
    }
    if ((millis() - lastEngineDebounceTime) > (debounceDelay*10)) {
      if (((mcpAValue & 0x4000) >> 14) == 1) {
        ecu_data.engineValueRAW = !ecu_data.engineValueRAW;
      }
    }
    
    if(ecu_data.hornValueRAW != ecu_data_old.hornValueRAW) {
      lastHornDebounceTime = millis();
    }
    if ((millis() - lastHornDebounceTime) > debounceDelay) {
      ecu_data.hornValueRAW = ((mcpAValue & 0x0080) >> 7);
    }
    
    if(ecu_data.parkingValueRAW != ecu_data_old.parkingValueRAW) {
      lastPBrakeDebounceTime = millis();
    }
    if ((millis() - lastPBrakeDebounceTime) > (debounceDelay*10)) {
      if (((mcpAValue & 0x8000) >> 15) == 1) {
        ecu_data.parkingValueRAW = !ecu_data.parkingValueRAW;
      }
    }

    if(ecu_data.doorLockValueRAW != ecu_data_old.doorLockValueRAW) {
      lastLockDebounceTime = millis();
    }

    if ((millis() - lastLockDebounceTime) > debounceDelay) {
      boolean lLock = ((mcpBValue & 0x0004) >> 2);
      boolean rLock = ((mcpBValue & 0x0008) >> 3);
      if (lLock | rLock) {
        ecu_data.doorLockValueRAW = (ecu_data.doorLockValueRAW ^ lLock)^(rLock << 1);
      }
    }
    
    if (gDebug) {
      DEBUG_PORT.println(F("************************************************************"));
      DEBUG_PORT.print(F("mcpAValue          : "));
      DEBUG_PORT.printf("%04x",mcpAValue);
      DEBUG_PORT.println();
      DEBUG_PORT.print(F("mcpBValue          : "));
      DEBUG_PORT.printf("%04x",mcpBValue);
      DEBUG_PORT.println();
      DEBUG_PORT.println(F("************************************************************"));
      DEBUG_PORT.print(F("brake              : "));
      DEBUG_PORT.printf("%04x",ecu_data.brakeValueRAW);
      DEBUG_PORT.println();
      DEBUG_PORT.print(F("accelerator        : "));
      DEBUG_PORT.printf("%04x",ecu_data.acceleratorValueRAW);
      DEBUG_PORT.println();
      DEBUG_PORT.print(F("steering           : "));
      if (ecu_data.steeringValueRAW > 0) {
        DEBUG_PORT.print(F("+"));
      } else {
        DEBUG_PORT.print(F("-"));
      }
      //DEBUG_PORT.println(abs(ecu_data.steeringValueRAW));
      DEBUG_PORT.printf("%03x",abs(ecu_data.steeringValueRAW));
      DEBUG_PORT.println();
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
} // chassisECUdata()

//**************************************************
// bodyECUdata()
//**************************************************

void bodyECUdata() {
  ecu_data.turnSignalIndicatorRAW = ecu_data.turnSwitchValueRAW;
  if (((ecu_data.turnSignalIndicatorRAW & 0x04) >> 2) == 1) {
    ecu_data.turnSignalIndicatorRAW = 0x03;
  }
  ecu_data.frontWiperStatusRAW = ecu_data.wiperFSwValueRAW;
  ecu_data.rearWiperStatusRAW = ecu_data.wiperRSwValueRAW;
  ecu_data.lightIndicatorRAW = (ecu_data.lightSwValueRAW|(ecu_data.lightFlValueRAW<<2));
  ecu_data.doorLockStatusRAW = (ecu_data.doorLockStatusRAW & 0x0c)|(ecu_data.doorLockValueRAW & 0x03);
  dispNext();
} // bodyECUdata()

//**************************************************
// ecu_setup()
//**************************************************

void ecu_setup() {
  if (ecuNumber==0) {
    //100
    define_rep_msg(0,brakeOutputIndMSG,8,1,1,10,1);
    define_rep_msg(0,throttlePositionMSG,8,1,1,10,2);
    define_rep_msg(0,engineRpmMSG,8,1,1,10,3);
    define_rep_msg(0,powerSteeringOutIndMSG,8,1,1,10,4);
    define_rep_msg(0,shiftPositionMSG,8,1,1,10,5);
    //20
    define_rep_msg(0,brakeOilIndMSG,8,1,1,50,6);
    define_rep_msg(0,absBrakeOperationMSG,8,1,1,50,7);
    define_rep_msg(0,throttleAdjustmentMSG,8,1,1,50,8);
    define_rep_msg(0,engineCoolantTempMSG,8,1,1,50,9);
    define_rep_msg(0,engineMalfunctionMSG,8,1,1,50,10);
    define_rep_msg(0,powerSteeringMalfMSG,8,1,1,50,11);
    define_rep_msg(0,engineStatusMSG,8,1,1,50,12);
    define_rep_msg(0,parkingBrakeStatusMSG,8,1,1,50,13);
    //2
    define_rep_msg(0,fuelAmountMSG,8,1,1,500,14);
    define_rep_msg(0,batteryWarningMSG,8,1,1,500,15);
    define_rep_msg(0,ecoDrivingJudgementMSG,8,1,1,500,16);
  } else if (ecuNumber==1) {
    //100
    define_rep_msg(0,brakeOperationMSG,8,1,1,10,1);
    define_rep_msg(0,accelerationOperationMSG,8,1,1,10,2);
    define_rep_msg(0,steeringWheelPosMSG,8,1,1,10,3);
    define_rep_msg(0,shiftPositionSwitchMSG,8,1,1,10,4);
    define_rep_msg(0,engineStartMSG,8,1,1,10,5);
    define_rep_msg(0,turnSwitchMSG,8,1,1,10,6);
    define_rep_msg(0,hornSwitchMSG,8,1,1,10,7);
    //20
    define_rep_msg(0,lightSwitchMSG,8,1,1,50,8);
    define_rep_msg(0,lightFlashMSG,8,1,1,50,9);
    define_rep_msg(0,parkingBrakeMSG,8,1,1,50,10);
    //10
    define_rep_msg(0,wiperSwitchFrontMSG,8,1,1,100,11);
    define_rep_msg(0,wiperSwitchRearMSG,8,1,1,100,12);
    define_rep_msg(0,doorLockUnlockMSG,8,1,1,100,13);
    define_rep_msg(0,lDoorSwWindowMSG,8,1,1,100,14);
    define_rep_msg(0,rDoorSwWindowMSG,8,1,1,100,15);
  } else if (ecuNumber==2) {
    //100
    define_rep_msg(0,turnSignalIndicatorMSG,8,1,1,10,1);
    define_rep_msg(0,hornOperationMSG,8,1,1,10,2);
    define_rep_msg(0,airbagActivationMSG,8,1,1,10,3);
    //20
    define_rep_msg(0,lightIndicatorMSG,8,1,1,50,4);
    //10
    define_rep_msg(0,frontWiperStatusMSG,8,1,1,100,5);
    define_rep_msg(0,rearWiperStatusMSG,8,1,1,100,6);
    define_rep_msg(0,doorLockStatusMSG,8,1,1,100,7);
    define_rep_msg(0,lDoorPositionMSG,8,1,1,100,8);
    define_rep_msg(0,rDoorPositionMSG,8,1,1,100,9);
    //2
    define_rep_msg(0,doorDriveUnitMalfuncMSG,8,1,1,500,10);
    define_rep_msg(0,seatBeltSensorMSG,8,1,1,500,11);
    define_rep_msg(0,seatBeltAlarmMSG,8,1,1,500,12);
    define_rep_msg(0,bonnetOpenSwitchMSG,8,1,1,500,13);
    define_rep_msg(0,trunkOpenSwitchMSG,8,1,1,500,14);
  }

  // Setup MCP23017s
  if (ecuNumber<2) {
    mcpA.begin_I2C(0x20);
    for (int i=0; i<16; i++) {
      mcpA.pinMode(i,INPUT_PULLUP);
      //mcpA.pullUp(i,HIGH);
    }
  } 
  if (ecuNumber==1) {
    // Setup MCP23017s
    mcpB.begin_I2C(0x21);
    for (int i=0; i<16; i++) {
      mcpB.pinMode(i,INPUT_PULLUP);
      //mcpB.pullUp(i,HIGH);
    }
    mcpB.pinMode(7,OUTPUT);
    mcpB.pinMode(14,OUTPUT);
    mcpB.pinMode(15,OUTPUT);
    mcpB.digitalWrite(7,LOW);
    mcpB.digitalWrite(14,LOW);
    mcpB.digitalWrite(15,LOW);
  }
  
  // Setup Timers
  if (ecuNumber<3) {
    my100Timer.begin(b100Set, 10000);  // Timer to run every 0.01 seconds
    my20Timer.begin (b20Set , 50000);  // Timer to run every 0.05 seconds
    if (ecuNumber!=0) {
      my10Timer.begin (b10Set ,100000);  // Timer to run every 0.10 seconds
    }
    if (ecuNumber!=1) {
      my2Timer.begin  (b2Set  ,500000);  // Timer to run every 0.50 seconds
    }
    // Setup LED
    pinMode(NEO_PIN,OUTPUT);
    digitalWrite(NEO_PIN,LOW);
  } else {
    // Setup NEOPIXELS
    pixels.begin();
    pixels.setBrightness(127);
  }
} // ecu_setup()

//**************************************************
// can_setup()
//**************************************************

void can_setup() {
  // Setup on chip CAN
  Can0.begin();
  Can0.setBaudRate(CANBUSSPEED0);
  Can0.setClock(CLK_60MHz);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff0);
  //Can0.mailboxStatus();
  if (ecuNumber==3) {
    Can1.begin();
    Can1.setBaudRate(CANBUSSPEED1);
    Can1.setClock(CLK_60MHz);
    Can1.setMaxMB(16);
    Can1.enableFIFO();
    Can1.enableFIFOInterrupt();
    Can1.onReceive(canSniff1);
    //Can1.mailboxStatus();
    Can2.begin();
    Can2.setBaudRate(CANBUSSPEED2);
    Can2.setClock(CLK_60MHz);
    Can2.setMaxMB(16);
    Can2.enableFIFO();
    Can2.enableFIFOInterrupt();
    Can2.onReceive(canSniff2);
    //Can2.mailboxStatus();
    // Setup MCP2515 CAN
    pinMode(CAN3_INT, INPUT);                     // Configuring pin for /INT input
#if MCPADDPINS
    DEBUG_PORT.print("*\tTXnBUF\t\tTRUE!\t\t\t *\r\n");
    pixels.setPixelColor(0, pixels.Color(0,255,0));
#else
    pixels.setPixelColor(0, pixels.Color(0,0,255));
#endif
    pixels.show();
    if (CANMCP3.begin(MCP_ANY, CAN3_SPEED, MCP_8MHZ) == CAN_OK){
      DEBUG_PORT.print(F("*\tCAN3:\t"));
      if (CAN3_SPEED == 12) {
        DEBUG_PORT.print(F("500kbps"));
      } else if (CAN3_SPEED == 9) {
        DEBUG_PORT.print(F("125kbps"));
      }
      DEBUG_PORT.print(F("\tInit OK!        \t *\r\n"));
      CANMCP3.setMode(MCP_NORMAL);
    } else {
      DEBUG_PORT.print(F("CAN3: Init Fail!!!\t *\r\n"));
    }
    DEBUG_PORT.print(F("**************************************************\r\n"));
  }
} // can_setup()

//**************************************************
// serial_setup()
//**************************************************

void serial_setup() {
  DEBUG_PORT.begin(500000);
  if (ecuNumber==0) {
    BT_CAR_PORT.begin(9600);
  }
  if (ecuNumber<3) {
    nextion_setup();
  }  
} // serial_setup()

//**************************************************
// get_ecuNumber()
//**************************************************

void get_ecuNumber() {
  pinMode(ECU_NUM_LO,INPUT_PULLUP);
  pinMode(ECU_NUM_HI,INPUT_PULLUP);
  while (millis()<500) {
    // do nothing
  }
  ecuNumber = (digitalRead(ECU_NUM_HI) << 1);
  ecuNumber += digitalRead(ECU_NUM_LO);
  DEBUG_PORT.println();
  DEBUG_PORT.print(F("**************************************************\r\n"));
  DEBUG_PORT.print(F("** VERSION\t\t"));
  DEBUG_PORT.print(strVERSION);
  DEBUG_PORT.print(F("\t\t**\r\n"));
  DEBUG_PORT.print(F("** https://github.com/mintynet/value-pasta-auto **\r\n"));
  DEBUG_PORT.print(F("** https://mintynet.com                         **\r\n"));
  DEBUG_PORT.print(F("**************************************************\r\n"));
  DEBUG_PORT.print(F("*\tecuNumber:\t"));
  if (ecuNumber<2) DEBUG_PORT.print(F("0"));
  DEBUG_PORT.print(ecuNumber,BIN);
  switch (ecuNumber) {
    case 0:
      DEBUG_PORT.print(F("\tPOWERTRAIN"));
      break;
    case 1:
      DEBUG_PORT.print(F("\tCHASSIS   "));
      break;
    case 2:
      DEBUG_PORT.print(F("\tBODY      "));
      break;
    case 3:
      DEBUG_PORT.print(F("\tGATEWAY   "));
      break;
    default:
      break;
  }
  DEBUG_PORT.print(F("\t *\r\n"));
  DEBUG_PORT.print(F("**************************************************\r\n"));
} // get_ecuNumber()

//**************************************************
// init_values()
//**************************************************

void init_values() {
  if (ecuNumber==0) {
    ecu_data.shiftPositionVal = 0x01;
    ecu_data.shiftPositionRAW = 0x01;
    ecu_data.engineStatusRAW = 0;
    ecu_data.engineCoolantTempRAW = 140;
    ecu_data.fuelAmountRAW = 43;
  } else if (ecuNumber==1) {
    preLightBit0 = mcpA.digitalRead(0);
    preWiperFrBit0 = mcpA.digitalRead(8);
    ecu_data.engineValueRAW = 0;
    ecu_data.parkingValueRAW = 1;
  }
} // init_values()

//**************************************************
// setup()
//**************************************************

void setup() {
  get_ecuNumber();
  serial_setup();
  while (millis()<10000) {
    // do nothing
  }
  can_setup();
  ecu_setup();
  init_values();
} // setup()

//**************************************************
// loop()
//**************************************************

void loop() {
  serialMenu();
  if (ecuNumber==0) {
    Can0.events();
    checkCanValues();
    powertrainECUdata();
    btSend();
    while (BT_CAR_PORT.available()) {
      char inChar = (char)BT_CAR_PORT.read();
      DEBUG_PORT.print(inChar);
  //    inputStringBLE += inChar;
  //    if (inChar == '\n') {
  //      stringComplete = true;
  //    }
    }
    updateCanValues();
  
    if (b100Hz) {
      can100Hz();
    }
    if (b20Hz) {
      can20Hz();
    }
    if (b10Hz) {
      can10Hz();
    }
    if (b2Hz) {
      can2Hz();
    }
    checkNext();
  } else if (ecuNumber==1){
    Can0.events();
    checkCanValues();
    chassisECUdata();
    updateCanValues();
  
    if (b100Hz) {
      can100Hz();
    }
    if (b20Hz) {
      can20Hz();
    }
    if (b10Hz) {
      can10Hz();
    }
    if (b2Hz) {
      can2Hz();
    }
    checkNext();
  } else if (ecuNumber==2) {
    Can0.events();
    checkCanValues();
    bodyECUdata();
    updateCanValues();
  
    if (b100Hz) {
      can100Hz();
    }
    if (b20Hz) {
      can20Hz();
    }
    if (b10Hz) {
      can10Hz();
    }
    if (b2Hz) {
      can2Hz();
    }
    checkNext();
  } else if (ecuNumber==3) {
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

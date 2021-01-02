#include <mcp_can.h>
#include <SPI.h>
boolean showDebug = false;
boolean showFDebug = false;
boolean firewallOpen = true;
unsigned long unlockId = 0x123;
unsigned long lockId = 0x124;
byte unlockBuf[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
byte len0,len1,len2;
byte rxBuf0[8],rxBuf1[8],rxBuf2[8];
char msgString[128];                            // Array to store serial string
#define             CAN3_INT    9               // Set INT to pin 9
#define             CAN3_CS     10              // Set INT to pin 10
#define             CAN3_SPEED  CAN_500KBPS     // 500kbps
MCP_CAN CANMCP3(CAN3_CS);                       // CAN1 interface using CS on digital pin 10

#include <FlexCAN_T4.h>
// 0 Powertrain
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
// 1 Chassis
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
// 2 Body
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can2;

#define DEBUG_PORT    Serial

const byte ledDisp = 4;

// 0 Powertrain
int CANBUSSPEED0  = 500000;
// 1 Chassis
int CANBUSSPEED1  = 500000;
// 2 Body
int CANBUSSPEED2  = 500000;
// 3 OBD2
int CANBUSSPEED3  = 500000;

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

//**************************************************
// canSniff0() Powertrain
//**************************************************

void canSniff0(const CAN_message_t &msg0) {
  if (showDebug) DEBUG_PORT.println("RX CAN0");
  int msgID = msg0.id;
  switch (msgID) {
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
  if (firewallOpen) {
    len0 = msg0.len;
    for(int i=0;i<len0;i++){
      rxBuf0[i]=msg0.buf[i];
    }
    CANMCP3.sendMsgBuf(msgID, 0, len0, rxBuf0);
    if (showDebug) {
      if (showFDebug) {
        DEBUG_PORT.print("Msg:");
        DEBUG_PORT.print(msgID,HEX);
        DEBUG_PORT.print("\tLen: ");
        DEBUG_PORT.print(len0);
        DEBUG_PORT.print(" Data:");
        for(int i=0;i<len0;i++) {
          if(rxBuf0[i]<16) {
            DEBUG_PORT.print("0");
          }
          DEBUG_PORT.print(rxBuf0[i],HEX);
          if(i<len0-1){
            DEBUG_PORT.print(":");
          }
        }
        DEBUG_PORT.println();
      }
      DEBUG_PORT.println("CAN1 to OBD2");
    }
  }
  digitalWrite(ledDisp,!digitalRead(ledDisp));
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
  if (showDebug) DEBUG_PORT.println("RX CAN1");
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
  if (firewallOpen) {
    len1 = msg1.len;
    for(int i=0;i<len1;i++){
      rxBuf1[i]=msg1.buf[i];
    }
    CANMCP3.sendMsgBuf(msgID, 0, len1, rxBuf1);
    if (showDebug) {
      if (showFDebug) {
        DEBUG_PORT.print("Msg:");
        DEBUG_PORT.print(msgID,HEX);
        DEBUG_PORT.print("\tLen: ");
        DEBUG_PORT.print(len1);
        DEBUG_PORT.print(" Data:");
        for(int i=0;i<len1;i++) {
          if(rxBuf1[i]<16) {
            DEBUG_PORT.print("0");
          }
          DEBUG_PORT.print(rxBuf1[i],HEX);
          if(i<len1-1){
            DEBUG_PORT.print(":");
          }
        }
        DEBUG_PORT.println();
      }
      DEBUG_PORT.println("CAN1 to OBD2");
    }
  }
  digitalWrite(ledDisp,!digitalRead(ledDisp));
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
  if (showDebug) DEBUG_PORT.println("RX CAN2");
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
  if (firewallOpen) {
    len2 = msg2.len;
    for(int i=0;i<len2;i++){
      rxBuf2[i]=msg2.buf[i];
    }
    CANMCP3.sendMsgBuf(msgID, 0, len2, rxBuf2);
    if (showDebug) {
      if (showFDebug) {
        DEBUG_PORT.print("Msg:");
        DEBUG_PORT.print(msgID,HEX);
        DEBUG_PORT.print("\tLen: ");
        DEBUG_PORT.print(len2);
        DEBUG_PORT.print(" Data:");
        for(int i=0;i<len2;i++) {
          if(rxBuf2[i]<16) {
            DEBUG_PORT.print("0");
          }
          DEBUG_PORT.print(rxBuf2[i],HEX);
          if(i<len2-1){
            DEBUG_PORT.print(":");
          }
        }
        DEBUG_PORT.println();
      }
      DEBUG_PORT.println("CAN2 to OBD2");
    }
  }
  digitalWrite(ledDisp,!digitalRead(ledDisp));
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
// serialMenu()
//**************************************************

static void serialMenu() {
  if (DEBUG_PORT.available()) {
    char ser = DEBUG_PORT.read();
    switch (ser)
    {
      case 'd':     // debug toggle
        showDebug = !showDebug;
        break;
      case 'D':     // full debug toggle
        showFDebug = !showFDebug;
        break;
      case 'f':     // firewall toggle
        firewallOpen = !firewallOpen;
        break;
      default:
        break;
    }
  }
} //serialMenu()

//**************************************************
// setup
//**************************************************

void setup() {
  // Setup Serial port
  DEBUG_PORT.begin(500000);
  // Setup CAN
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
  pinMode(CAN3_INT, INPUT);                     // Configuring pin for /INT input
  if(CANMCP3.begin(MCP_ANY, CAN3_SPEED, MCP_8MHZ) == CAN_OK){
  DEBUG_PORT.print("CAN3:\t");
  if (CAN3_SPEED == 12) {
    DEBUG_PORT.print("500kbps");
  } else if (CAN3_SPEED == 9) {
    DEBUG_PORT.print("125kbps");
  }
  DEBUG_PORT.print("\tInit OK!\r\n");
  CANMCP3.setMode(MCP_NORMAL);
  } else DEBUG_PORT.print("CAN3: Init Fail!!!\r\n");
  while (millis()<10000) {
    // do nothing
  }
  // Setup led Pin
  pinMode(ledDisp,OUTPUT);
} // setup()

//**************************************************
// loop
//**************************************************

void loop() {
  serialMenu();
  Can0.events();
  Can1.events();
  Can2.events();
} // loop()

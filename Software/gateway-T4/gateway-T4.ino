                                                  // Teensyduino 1.53
                                                  // Arduino 1.8.13
#include "mcp_minty.h"
#include "mcp_can.h"                              // version 1.5 25/09/17 from https://github.com/coryjfowler/MCP_CAN_lib modified for 10MHz SPI
#include <SPI.h>                                  // version 1.0
#include <Adafruit_NeoPixel.h>                    // version 1.7.0
#include <FlexCAN_T4.h>                           // version 2018
boolean       gDebug            = false;
boolean       proofDebug        = true;
boolean       firewallOpen0     = false;
boolean       firewallOpen1     = false;
boolean       firewallOpen2     = false;

const unsigned long unlockId    = 0x123;
const unsigned long lockId      = 0x124;
byte unlockBuf[8]               = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};

long unsigned int   rxId;                         // Used for MCP3 received msgs
unsigned char       len         = 0;              // Used for MCP3 received msgs
unsigned char       rxBuf[8];                     // Used for MCP3 received msgs
#define             MAX_PIXELS  1                 // Number of Neopixels
#define             CAN3_INT    9                 // Set INT to pin 9
#define             CAN3_CS     10                // Set INT to pin 10
#define             CAN3_SPEED  CAN_500KBPS       // 500kbps
/*
#define             CAN3_TX0BUF 24                // TX0 RTS Pin
#define             CAN3_TX1BUF 25                // TX1 RTS Pin
#define             CAN3_TX2BUF 26                // TX2 RTS Pin
#define             CAN3_RX0BF  27                // RX0 INT Pin
#define             CAN3_RX1BF  28                // RX1 INT Pin
*/
MCP_CAN             CANMCP3(CAN3_CS);             // CAN3 interface using CS on digital pin 10

unsigned int        cnt0  = 0;
unsigned int        cnt1  = 0;
unsigned int        cnt2  = 0;
unsigned int        cnt30 = 0;
unsigned int        cnt31 = 0;
unsigned int        cnt32 = 0;

// 0 Powertrain
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
// 1 Chassis
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
// 2 Body
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can2;

#define DEBUG_PORT    Serial

const byte NEO_PIN = 4;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(MAX_PIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

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
//reset message
const unsigned int resetMSG                 = 0x280;

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
    byte sndStat = sendTX0(msg0.id,msg0.len,msg0.buf,1);
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
    byte sndStat = sendTX0(msg1.id,msg1.len,msg1.buf,1);
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
    byte sndStat = sendTX0(msg2.id,msg2.len,msg2.buf,1);
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
// serialMenu()
//**************************************************

static void serialMenu() {
  CAN_message_t msg;
  if (DEBUG_PORT.available()) {
    char ser = DEBUG_PORT.read();
    switch (ser)
    {
      case '1':     // reboot body ECU
        msg.id = resetMSG;
        msg.len = 0x0;
        Can2.write(msg);
        break;
      case '2':     // reboot chassis ECU
        msg.id = resetMSG;
        msg.len = 0x0;
        Can1.write(msg);
        break;
      case '3':     // reboot body and chassis ECU
        msg.id = resetMSG;
        msg.len = 0x0;
        Can2.write(msg);
        delay(50);
        Can1.write(msg);
        break;
      case '4':     // reboot powertrain ECU
        msg.id = resetMSG;
        msg.len = 0x0;
        Can0.write(msg);
        break;
      case '5':     // reboot body and powertrain ECU
        msg.id = resetMSG;
        msg.len = 0x0;
        Can2.write(msg);
        delay(50);
        Can0.write(msg);
        break;
      case '6':     // reboot chassis and powertrain ECU
        msg.id = resetMSG;
        msg.len = 0x0;
        Can1.write(msg);
        delay(50);
        Can0.write(msg);
        break;
      case '7':     // reboot body, chassis and powertrain ECU
        msg.id = resetMSG;
        msg.len = 0x0;
        Can2.write(msg);
        delay(50);
        Can1.write(msg);
        delay(50);
        Can0.write(msg);
        break;
      case '8':     // reboot gateway ECU
        SCB_AIRCR = 0x05FA0004;
        break;
      case 'F':     // reboot all ECUs
        msg.id = resetMSG;
        msg.len = 0x0;
        Can2.write(msg);
        delay(50);
        Can1.write(msg);
        delay(50);
        Can0.write(msg);
        delay(50);
        SCB_AIRCR = 0x05FA0004;
        break;
      case 'd':     // toggle DEBUG
        gDebug=!gDebug;
        break;
      case 'h':     // help
        DEBUG_PORT.println(F("f\tToggle firewall"));
        DEBUG_PORT.println(F("1\tReboot Body ECU"));
        DEBUG_PORT.println(F("2\tReboot Chassis ECU"));
        DEBUG_PORT.println(F("4\tReboot Powertrain ECU"));
        DEBUG_PORT.println(F("8\tReboot Gateway ECU"));
        break;
      case 'f':     // firewall toggle
        firewallOpen0 = !firewallOpen0;
        firewallOpen1 = !firewallOpen1;
        firewallOpen2 = !firewallOpen2;
        if (firewallOpen0) {
          DEBUG_PORT.println(F("Firewall 0 Open"));
        } else {
          DEBUG_PORT.println(F("Firewall 0 Closed"));
        }
        if (firewallOpen1) {
          DEBUG_PORT.println(F("Firewall 1 Open"));
        } else {
          DEBUG_PORT.println(F("Firewall 1 Closed"));
        }
        if (firewallOpen2) {
          DEBUG_PORT.println(F("Firewall 2 Open"));
        } else {
          DEBUG_PORT.println(F("Firewall 2 Closed"));
        }
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
  // Setup NEOPIXELS
  pixels.begin();
  pixels.setBrightness(127);
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
  //Can1.mailboxStatus();
  //Can2.mailboxStatus();

/*  pinMode(CAN3_INT, INPUT);                     // Configuring pin for /INT input
  pinMode(CAN3_TX0BUF, OUTPUT);                 // Configuring pin for TX0 Buffer
  digitalWrite(CAN3_TX0BUF,HIGH);               // Set TX0 Buffer pin HIGH
  pinMode(CAN3_TX1BUF, OUTPUT);                 // Configuring pin for TX1 Buffer
  digitalWrite(CAN3_TX1BUF,HIGH);               // Set TX1 Buffer pin HIGH
  pinMode(CAN3_TX2BUF, OUTPUT);                 // Configuring pin for TX2 Buffer
  digitalWrite(CAN3_TX2BUF,HIGH);               // Set TX2 Buffer pin HIGH */
  
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
} // setup()

//**************************************************
// loop
//**************************************************

void loop() {
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
  if(!digitalRead(CAN3_INT)) {
    CANMCP3.readMsgBuf(&rxId, &len, rxBuf);
    switch (rxId) {
      case unlockId:
        if((rxBuf[1] == unlockBuf[1])&(rxBuf[2] == unlockBuf[2])&(rxBuf[3] == unlockBuf[3])&(rxBuf[4] == unlockBuf[4])&(rxBuf[5] == unlockBuf[5])&(rxBuf[6] == unlockBuf[6])&(rxBuf[7] == unlockBuf[7])) {
          if((rxBuf[0]&0x01)>>0) {
            firewallOpen0 = true;
            DEBUG_PORT.println(F("Firewall0 unlock"));
          }
          if((rxBuf[0]&0x02)>>1) {
            firewallOpen1 = true;
            DEBUG_PORT.println(F("Firewall1 unlock"));
          }
          if((rxBuf[0]&0x04)>>2) {
            firewallOpen2 = true;
            DEBUG_PORT.println(F("Firewall2 unlock"));
          }
        }
        break;
      case lockId:
        if((rxBuf[1] == unlockBuf[1])&(rxBuf[2] == unlockBuf[2])&(rxBuf[3] == unlockBuf[3])&(rxBuf[4] == unlockBuf[4])&(rxBuf[5] == unlockBuf[5])&(rxBuf[6] == unlockBuf[6])&(rxBuf[7] == unlockBuf[7])) {
          if((rxBuf[0]&0x01)>>0) {
            firewallOpen0 = false;
            DEBUG_PORT.println(F("Firewall0 lock"));         
          }
          if((rxBuf[0]&0x02)>>1) {
            firewallOpen1 = false;
            DEBUG_PORT.println(F("Firewall1 lock"));         
          }
          if((rxBuf[0]&0x04)>>2) {
            firewallOpen2 = false;
            DEBUG_PORT.println(F("Firewall2 lock"));         
          }
        }
        break;
      case resetMSG:
        SCB_AIRCR = 0x05FA0004;
        break;
      default:
        break;
    }
  }
  Can0.events();
  Can1.events();
  Can2.events();
  pixels.setPixelColor(0, pixels.Color(0,0,0));
  pixels.show();
} // loop()

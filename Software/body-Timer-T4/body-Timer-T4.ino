#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

#define DEBUG_PORT    Serial
#define NEXTION_PORT  Serial5

const byte ledDisp = 4;
const boolean gDebug = true;

static CAN_message_t msg;

int CANBUSSPEED = 500000;
byte nextPage = 0;

IntervalTimer my100Timer;
IntervalTimer my20Timer;
IntervalTimer my10Timer;
IntervalTimer my2Timer;

boolean b100Hz  = 0;
boolean b20Hz   = 0;
boolean b10Hz   = 0;
boolean b2Hz    = 0;

const byte msgSpacing = 200;
long lastCanMsg = 0;

//100Hz Body Output
const unsigned int turnSignalIndicatorMSG   = 0x8d;
const unsigned int hornOperationMSG         = 0xa2;
const unsigned int airbagActivationMSG      = 0xb4;
//100Hz Chassis Input
const unsigned int turnSwitchMSG            = 0x83;
const unsigned int hornSwitchMSG            = 0x98;
const unsigned int brakeOperationMSG        = 0x1a;
//20Hz Body Output
const unsigned int lightIndicatorMSG        = 0x1bb;
//20z Chassis Input
const unsigned int lightSwitchMSG           = 0x1a7;
const unsigned int lightFlashMSG            = 0x1b1;
const unsigned int parkingBrakeMSG          = 0x1c9;
//10Hz Body Output
const unsigned int frontWiperStatusMSG      = 0x266;
const unsigned int rearWiperStatusMSG       = 0x27b;
const unsigned int doorLockStatusMSG        = 0x290;
const unsigned int lDoorPositionMSG         = 0x2bb;
const unsigned int rDoorPositionMSG         = 0x2a6;
//10Hz Chassis Input
const unsigned int wiperSwitchFrontMSG      = 0x25c;
const unsigned int wiperSwitchRearMSG       = 0x271;
const unsigned int doorLockUnlockMSG        = 0x286;
const unsigned int lDoorSwWindowMSG         = 0x29c;
const unsigned int rDoorSwWindowMSG         = 0x2b1;
//2Hz Body Output
const unsigned int doorDriveUnitMalfuncMSG  = 0x420;
const unsigned int seatBeltSensorMSG        = 0x457;
const unsigned int seatBeltAlarmMSG         = 0x461;
const unsigned int bonnetOpenSwitchMSG      = 0x46c;
const unsigned int trunkOpenSwitchMSG       = 0x477;

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
} // can100Hz

//**************************************************
// can20Hz
//**************************************************

void can20Hz() {
  //noInterrupts();
  msg.id = lightIndicatorMSG;
  msg.len = 8;
  msg.buf[0] = ecu_data.lightIndicatorRAW;
  Can0.write(msg);
  b20Hz = 0;
  digitalWrite(ledDisp,!digitalRead(ledDisp));
  //interrupts();
} // can20Hz

//**************************************************
// can10Hz
//**************************************************

void can10Hz() {
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
} // can10Hz()

//**************************************************
// can2Hz
//**************************************************

void can2Hz() {
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
} // can2Hz()

//**************************************************
// canSniff()
//**************************************************

void canSniff(const CAN_message_t &msg) {
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
// setup
//**************************************************

void setup() {
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
} // setup()

//**************************************************
// loop
//**************************************************

void loop() {
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
} // loop()

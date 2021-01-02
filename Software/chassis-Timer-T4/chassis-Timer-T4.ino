#include <FlexCAN_T4.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <ResponsiveAnalogRead.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

Adafruit_MCP23017 mcpA, mcpB;

#define DEBUG_PORT    Serial
#define NEXTION_PORT  Serial5

const byte ana0x01A = A0;
const byte ana0x02F = A1;
const byte ana0x058 = A2;

const byte ledDisp = 4;
const boolean gDebug = true;

ResponsiveAnalogRead analog0(ana0x01A,true);
ResponsiveAnalogRead analog1(ana0x02F,true);
ResponsiveAnalogRead analog2(ana0x058,true);

static CAN_message_t msg;

int CANBUSSPEED = 500000;
byte nextPage = 0;

IntervalTimer my100Timer;
IntervalTimer my20Timer;
IntervalTimer my10Timer;

boolean b100Hz  = 0;
boolean b20Hz   = 0;
boolean b10Hz   = 0;

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

//100Hz Chassis Output
const unsigned int brakeOperationMSG        = 0x1a;
const unsigned int accelerationOperationMSG = 0x2f;
const unsigned int steeringWheelPosMSG      = 0x58;
const unsigned int shiftPositionSwitchMSG   = 0x6d;
const unsigned int engineStartMSG           = 0x1b8;
const unsigned int turnSwitchMSG            = 0x83;
const unsigned int hornSwitchMSG            = 0x98;
//100Hz Powertrain Input
const unsigned int brakeOutputIndMSG        = 0x24;
const unsigned int throttlePositionMSG      = 0x39;
const unsigned int engineRpmMSG             = 0x43;
const unsigned int powerSteeringOutIndMSG   = 0x62;
const unsigned int shiftPositionMSG         = 0x77;
//100Hz Body Input
const unsigned int turnSignalIndicatorMSG   = 0x8d;
const unsigned int hornOperationMSG         = 0xa2;
const unsigned int airbagActivationMSG      = 0xb4;
//20z Chassis Output
const unsigned int lightSwitchMSG           = 0x1a7;
const unsigned int lightFlashMSG            = 0x1b1;
const unsigned int parkingBrakeMSG          = 0x1c9;
//20Hz Powertrain Input
const unsigned int brakeOilIndMSG           = 0x146;
const unsigned int absBrakeOperationMSG     = 0x15a;
const unsigned int throttleAdjustmentMSG    = 0x16f;
const unsigned int engineCoolantTempMSG     = 0x183;
const unsigned int engineMalfunctionMSG     = 0x18d;
const unsigned int powerSteeringMalfMSG     = 0x198;
const unsigned int engineStatusMSG          = 0x19a;
const unsigned int parkingBrakeStatusMSG    = 0x1d3;
//20Hz Body Input
const unsigned int lightIndicatorMSG        = 0x1bb;
//10Hz Chassis Output
const unsigned int wiperSwitchFrontMSG      = 0x25c;
const unsigned int wiperSwitchRearMSG       = 0x271;
const unsigned int doorLockUnlockMSG        = 0x286;
const unsigned int lDoorSwWindowMSG         = 0x29c;
const unsigned int rDoorSwWindowMSG         = 0x2b1;
//10Hz Body Input
const unsigned int frontWiperStatusMSG      = 0x266;
const unsigned int rearWiperStatusMSG       = 0x27b;
const unsigned int doorLockStatusMSG        = 0x290;
const unsigned int lDoorPositionMSG         = 0x2bb;
const unsigned int rDoorPositionMSG         = 0x2a6;
//2Hz Powertrain Input
const unsigned int fuelAmountMSG            = 0x3d4;
const unsigned int batteryWarningMSG        = 0x3de;
const unsigned int ecoDrivingJudgementMSG   = 0x482;
//2Hz Body Input
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
// can100Hz
//**************************************************

void can100Hz() {
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
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = engineStartMSG;
  msg.buf[0] = ecu_data.engineValueRAW;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = turnSwitchMSG;
  msg.buf[0] = ecu_data.turnSwitchValueRAW;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = hornSwitchMSG;
  msg.buf[0] = ecu_data.hornValueRAW;
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
  msg.id = lightSwitchMSG;
  msg.len = 8;
  msg.buf[0] = ecu_data.lightSwValueRAW;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = lightFlashMSG;
  msg.buf[0] = ecu_data.lightFlValueRAW;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = parkingBrakeMSG;
  msg.buf[0] = ecu_data.parkingValueRAW;
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
  msg.id = wiperSwitchFrontMSG;
  msg.len = 8;
  msg.buf[0] = ecu_data.wiperFSwValueRAW;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = wiperSwitchRearMSG;
  msg.buf[0] = ecu_data.wiperRSwValueRAW;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = doorLockUnlockMSG;
  msg.buf[0] = ecu_data.doorLockValueRAW;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = lDoorSwWindowMSG;
  msg.buf[0] = ecu_data.lDoorSwValueRAW;
  Can0.write(msg);
  delayMicroseconds(msgSpacing);
  msg.id = rDoorSwWindowMSG;
  msg.buf[0] = ecu_data.rDoorSwValueRAW;
  Can0.write(msg);
  b10Hz = 0;
  digitalWrite(ledDisp,!digitalRead(ledDisp));
  //interrupts();
} // can10Hz()

//**************************************************
// canSniff()
//**************************************************

void canSniff(const CAN_message_t &msg) {
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
  Can0.setClock(CLK_60MHz);
  Can0.setBaudRate(CANBUSSPEED);
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
} // setup()

//**************************************************
// loop
//**************************************************

void loop() {
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
      DEBUG_PORT.println("************************************************************");
      DEBUG_PORT.print("mcpAValue          : ");
      if (mcpAValue < 0x10) DEBUG_PORT.print("0");
      if (mcpAValue < 0x100) DEBUG_PORT.print("0");
      if (mcpAValue < 0x1000) DEBUG_PORT.print("0");
      DEBUG_PORT.println(mcpAValue,HEX);
      DEBUG_PORT.print("mcpBValue          : ");
      if (mcpBValue < 0x10) DEBUG_PORT.print("0");
      if (mcpBValue < 0x100) DEBUG_PORT.print("0");
      if (mcpBValue < 0x1000) DEBUG_PORT.print("0");
      DEBUG_PORT.println(mcpBValue,HEX);
      DEBUG_PORT.println("************************************************************");
      DEBUG_PORT.print("brake              : ");
      if (ecu_data.brakeValueRAW < 10) DEBUG_PORT.print("0");
      if (ecu_data.brakeValueRAW < 100) DEBUG_PORT.print("0");
      if (ecu_data.brakeValueRAW < 1000) DEBUG_PORT.print("0");
      DEBUG_PORT.println(ecu_data.brakeValueRAW);
      DEBUG_PORT.print("accelerator        : ");
      if (ecu_data.acceleratorValueRAW < 10) DEBUG_PORT.print("0");
      if (ecu_data.acceleratorValueRAW < 100) DEBUG_PORT.print("0");
      if (ecu_data.acceleratorValueRAW < 1000) DEBUG_PORT.print("0");
      DEBUG_PORT.println(ecu_data.acceleratorValueRAW);
      DEBUG_PORT.print("steering           : ");
      if (ecu_data.steeringValueRAW > 0) {
        DEBUG_PORT.print("+");
        if (ecu_data.steeringValueRAW < 10) DEBUG_PORT.print("0");
        if (ecu_data.steeringValueRAW < 100) DEBUG_PORT.print("0");
      } else {
        DEBUG_PORT.print("-");
        if (ecu_data.steeringValueRAW > -10) DEBUG_PORT.print("0");
        if (ecu_data.steeringValueRAW > -100) DEBUG_PORT.print("0");
      }
      DEBUG_PORT.println(abs(ecu_data.steeringValueRAW));
      DEBUG_PORT.println("************************************************************");
      DEBUG_PORT.print("shiftValueRAW        : ");
      DEBUG_PORT.println(ecu_data.shiftValueRAW,HEX);
      DEBUG_PORT.print("engineValueRAW       : ");
      DEBUG_PORT.println(ecu_data.engineValueRAW,HEX);
      DEBUG_PORT.print("engineStatusRAW      : ");
      DEBUG_PORT.println(ecu_data.engineStatusRAW,HEX);
      DEBUG_PORT.print("engineMalfunctionRAW : ");
      DEBUG_PORT.println(ecu_data.engineMalfunctionRAW,HEX);
      DEBUG_PORT.print("hazardValueRAW       : ");
      DEBUG_PORT.println(ecu_data.hazardValueRAW,HEX);
      DEBUG_PORT.print("turnSwitchValueRAW   : ");
      DEBUG_PORT.println(ecu_data.turnSwitchValueRAW,HEX);
      DEBUG_PORT.print("hornValueRAW         : ");
      DEBUG_PORT.println(ecu_data.hornValueRAW,HEX);
      DEBUG_PORT.print("Light            Dir : ");
      DEBUG_PORT.print(curLightDir);
      DEBUG_PORT.print(" | counter: ");
      DEBUG_PORT.println(lightCount);
      DEBUG_PORT.print("lightSwValueRAW      : ");
      DEBUG_PORT.println(ecu_data.lightSwValueRAW,HEX);
      DEBUG_PORT.print("lightFlValueRAW      : ");
      DEBUG_PORT.println(ecu_data.lightFlValueRAW,HEX);
      DEBUG_PORT.print("parkingValueRAW      : ");
      DEBUG_PORT.println(ecu_data.parkingValueRAW,HEX);
      DEBUG_PORT.print("FrWiper          Dir : ");
      DEBUG_PORT.print(curWiperFrDir);
      DEBUG_PORT.print(" | counter: ");
      DEBUG_PORT.println(wiperFrCount);
      DEBUG_PORT.print("wiperFSwValueRAW     : ");
      DEBUG_PORT.println(ecu_data.wiperFSwValueRAW,HEX);
      DEBUG_PORT.print("wiperRSwValueRAW     : ");
      DEBUG_PORT.println(ecu_data.wiperRSwValueRAW,HEX);
      DEBUG_PORT.print("doorLockValueRAW     : ");
      DEBUG_PORT.println(ecu_data.doorLockValueRAW,HEX);
      DEBUG_PORT.print("lDoorSwValueRAW      : ");
      DEBUG_PORT.println(ecu_data.lDoorSwValueRAW,HEX);
      DEBUG_PORT.print("rDoorSwValueRAW      : ");
      DEBUG_PORT.println(ecu_data.rDoorSwValueRAW,HEX);
      DEBUG_PORT.print("nextPage             : ");
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
} // loop()

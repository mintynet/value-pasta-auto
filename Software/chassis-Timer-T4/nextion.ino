//**************************************************
// endNextion
//**************************************************

void endNextion () {
  NEXTION_PORT.write(0xff);
  NEXTION_PORT.write(0xff);
  NEXTION_PORT.write(0xff);
} //endNextion()

//**************************************************
// dispSpeed
//**************************************************

void dispSpeed (long i) {
  const int speedomax = 120;  
  if (i > speedomax) {
    i = speedomax;
  }
  const int speedoSTangle = 270;
  const int speedoSUMangle = 270;
  long speedo;
  long speedoangle;
    speedo = i;
    speedoangle = (speedo*speedoSUMangle/speedomax)+speedoSTangle;
    if (speedoangle>359) 
    { 
      speedoangle = speedoangle - 360;
    }
    NEXTION_PORT.print(F("SPEED.val="));
    NEXTION_PORT.print(speedoangle);
    endNextion();
    NEXTION_PORT.print(F("nSPEED.val="));
    NEXTION_PORT.print(speedo);
    endNextion();
} //dispSpeed()

//**************************************************
// dispRev
//**************************************************

void dispRev (long i) {
  const int revmax = 8000;
  if (i > revmax) {
    i = revmax;
  }
  const int revSTangle = 270;
  const int revSUMangle = 270;
  long rev;
  long revangle;
  rev = i;
  revangle = (rev*revSUMangle/revmax)+revSTangle;
  if (revangle>359) 
  { 
    revangle = revangle - 360;
  }
  NEXTION_PORT.print(F("REV.val="));
  NEXTION_PORT.print(revangle);
  endNextion();
  NEXTION_PORT.print(F("nREV.val="));
  NEXTION_PORT.print(rev);
  endNextion();
} //dispRev()

//**************************************************
// dispTPS
//**************************************************

void dispTps (byte i) {
  NEXTION_PORT.print(F("jTPS.val="));
  if(i==0)
  {
    NEXTION_PORT.print(F("1"));
  } else if (i>100)
  {
    NEXTION_PORT.print(F("100"));
  } else
  {
    NEXTION_PORT.print(i);
  }
  endNextion();
  NEXTION_PORT.print(F("nTPS.val="));
  NEXTION_PORT.print(i);
  endNextion();
} //dispTps()

//**************************************************
// dispBrake
//**************************************************

void dispBrake (byte i) {
  NEXTION_PORT.print(F("jBRAKE.val="));
  if(i==0)
  {
    NEXTION_PORT.print(F("1"));
  } else if (i>100)
  {
    NEXTION_PORT.print(F("100"));
  } else
  {
    NEXTION_PORT.print(i);
  }
  endNextion();
  NEXTION_PORT.print(F("nBRAKE.val="));
  NEXTION_PORT.print(i);
  endNextion();
} //dispBrake()

//**************************************************
// dispChassis
//**************************************************

void dispChassis (long i,byte j,byte k,byte l) {
  unsigned int tempSlider;
  tempSlider = (k * 100/255);
  unsigned int fuelSlider;
  fuelSlider = (l * 100/45);
  const int revmax = 8000;
  if (i > revmax) {
    i = revmax;
  }
  const int revSTangle = 270;
  const int revSUMangle = 270;
  long rev;
  long revangle;
  rev = i;
  revangle = (rev*revSUMangle/revmax)+revSTangle;
  if (revangle>359) { 
    revangle = revangle - 360;
  }
  NEXTION_PORT.print(F("REV.val="));
  NEXTION_PORT.print(revangle);
  endNextion();
  NEXTION_PORT.print(F("nSPEED.val="));
  NEXTION_PORT.print(j);
  endNextion();
  NEXTION_PORT.print(F("vis nSPEED,1"));
  endNextion();
  NEXTION_PORT.print(F("vis tSPEED,1"));
  endNextion();
  NEXTION_PORT.print(F("tGEAR.txt=\""));
  NEXTION_PORT.print(ecu_data.shiftPositionVal);
  NEXTION_PORT.print(F("\""));
  endNextion();
  NEXTION_PORT.print(F("jTEMP.val="));
  NEXTION_PORT.print(tempSlider);
  endNextion();
  NEXTION_PORT.print(F("jFUEL.val="));
  NEXTION_PORT.print(fuelSlider);
  endNextion();
//  Serial.print(ecu_data.turnSignalIndicatorRAW);
  if (ecu_data.turnSignalIndicatorRAW==3) {
    NEXTION_PORT.print(F("p0.pic=12"));
    endNextion();
    NEXTION_PORT.print(F("flasher.val=2"));
    endNextion();
  } else if (ecu_data.turnSignalIndicatorRAW == 0) {
    NEXTION_PORT.print(F("flasher.val=0"));
    endNextion();
  } else {
    NEXTION_PORT.print(F("p0.pic=10"));
    endNextion();
    NEXTION_PORT.print(F("flasher.val=1"));
    endNextion();
  }
//  Serial.print(" : ");
//  Serial.println(ecu_data.lightIndicatorRAW);
  if ((ecu_data.lightIndicatorRAW & 0x04) >> 2) {
    NEXTION_PORT.print(F("vis p1,1"));
    endNextion();
    NEXTION_PORT.print(F("p1.pic=14"));
    endNextion();
  } else if ((ecu_data.lightIndicatorRAW & 0x02) >> 1) {
    NEXTION_PORT.print(F("vis p1,1"));
    endNextion();
    NEXTION_PORT.print(F("p1.pic=13"));
    endNextion();
  } else if ((ecu_data.lightIndicatorRAW & 0x01) >> 0) {
    NEXTION_PORT.print(F("vis p1,1"));
    endNextion();
    NEXTION_PORT.print(F("p1.pic=11"));
    endNextion();
  } else {
    NEXTION_PORT.print(F("vis p1,0"));
    endNextion();
  }
  NEXTION_PORT.print(F("vis p2,"));
  NEXTION_PORT.print(ecu_data.parkingBrakeStatusRAW);
  endNextion();

  if (ecu_data.engineMalfunctionRAW) {
    NEXTION_PORT.print(F("p3.pic=9"));
    endNextion();
    NEXTION_PORT.print(F("vis p3,"));
    NEXTION_PORT.print(ecu_data.engineMalfunctionRAW);
    endNextion();
  } else {
    if (ecu_data.bonnetOpenSwitchRAW) {
      NEXTION_PORT.print(F("p3.pic=24"));
      endNextion();
      NEXTION_PORT.print(F("vis p3,1"));
      endNextion();
    } else if (ecu_data.trunkOpenSwitchRAW) {
      NEXTION_PORT.print(F("p3.pic=25"));
      endNextion();
      NEXTION_PORT.print(F("vis p3,1"));
      endNextion();
    } else if (((ecu_data.doorLockStatusRAW & 0x04)>>2)&((ecu_data.doorLockStatusRAW & 0x08)>>3)) {
      NEXTION_PORT.print(F("p3.pic=21"));
      endNextion();
      NEXTION_PORT.print(F("vis p3,1"));
      endNextion();
    } else if ((ecu_data.doorLockStatusRAW & 0x04)>>2) {
      NEXTION_PORT.print(F("p3.pic=22"));
      endNextion();
      NEXTION_PORT.print(F("vis p3,1"));
      endNextion();
    } else if ((ecu_data.doorLockStatusRAW & 0x08)>>3) {
      NEXTION_PORT.print(F("p3.pic=23"));
      endNextion();
      NEXTION_PORT.print(F("vis p3,1"));
      endNextion();
    } else {
      NEXTION_PORT.print(F("vis p3,0"));
      endNextion();
    }
  }
} //dispChassis()

//**************************************************
// checkNext
//**************************************************

void checkNext() {
  byte next_resp[16];
  int next_index = 0;
  for (next_index = 0;next_index < 16; next_index++)
  {
    next_resp[next_index] = 0;
  }
  if (NEXTION_PORT.available())
  {
    for (next_index = 0;next_index < 16; next_index++)
    {
      next_resp[next_index] = NEXTION_PORT.read();
      if(gDebug) {
        if (next_resp[next_index] < 16) DEBUG_PORT.print(F("0"));
        DEBUG_PORT.print(next_resp[next_index],HEX);
      }
      delayMicroseconds(50);
    }
    if(gDebug) DEBUG_PORT.println();
  } else {
    return;
  }
  if((next_resp[0]==0x66) && (next_resp[1]==0x0)) 
  {
    nextPage=0;
    if(gDebug) DEBUG_PORT.println(F("CHANGE PAGE 0"));
  } else if((next_resp[0]==0x66) && (next_resp[1]==0x1)) 
  {
    nextPage=1;
    dispChassis(ecu_data.engineRpmRAW,abs(ecu_data.speedKphRAW*0.621371),ecu_data.engineCoolantTempRAW,ecu_data.fuelAmountRAW);
    if(gDebug) DEBUG_PORT.println(F("CHANGE PAGE 1"));
  } else if((next_resp[0]==0x66) && (next_resp[1]==0x2))
  {
    nextPage=2;
    dispSpeed(ecu_data.speedKphRAW*0.621371);
    if(gDebug) DEBUG_PORT.println(F("CHANGE PAGE 2"));
  } else if((next_resp[0]==0x66) && (next_resp[1]==0x3))
  {
    nextPage=3;
    dispRev(ecu_data.engineRpmRAW);
    if(gDebug) DEBUG_PORT.println(F("CHANGE PAGE 3"));
  } else if((next_resp[0]==0x66) && (next_resp[1]==0x4))
  {
    nextPage=4;
    dispTps(ecu_data.throttlePositionRAW*100/1024);
    dispBrake(ecu_data.brakeOutputRAW*100/1024);
    if(gDebug) DEBUG_PORT.println(F("CHANGE PAGE 4"));
  } 
} //checkNext

//**************************************************
// dispNext
//**************************************************

void dispNext() {
  if (nextPage==1) {
    if ((ecu_data.engineRpmRAW != ecu_data_old.engineRpmRAW)|
        (ecu_data.speedKphRAW != ecu_data_old.speedKphRAW)|
        (ecu_data.engineCoolantTempRAW != ecu_data_old.engineCoolantTempRAW)|
        (ecu_data.fuelAmountRAW != ecu_data_old.fuelAmountRAW)|
        (ecu_data.shiftPositionVal!=ecu_data_old.shiftPositionVal)|
        (ecu_data.turnSignalIndicatorRAW!=ecu_data_old.turnSignalIndicatorRAW)|
        (ecu_data.lightIndicatorRAW!=ecu_data_old.lightIndicatorRAW)|
        (ecu_data.parkingBrakeStatusRAW!=ecu_data_old.parkingBrakeStatusRAW)|
        (ecu_data.engineMalfunctionRAW!=ecu_data_old.engineMalfunctionRAW)|
        (ecu_data.bonnetOpenSwitchRAW!=ecu_data_old.bonnetOpenSwitchRAW)|
        (ecu_data.trunkOpenSwitchRAW!=ecu_data_old.trunkOpenSwitchRAW)|
        (ecu_data.doorLockStatusRAW!=ecu_data_old.doorLockStatusRAW)) {
      dispChassis(ecu_data.engineRpmRAW,abs(ecu_data.speedKphRAW*0.621371),ecu_data.engineCoolantTempRAW,ecu_data.fuelAmountRAW);
    }
  } else if (nextPage==2) {
    if (ecu_data.speedKphRAW != ecu_data_old.speedKphRAW) {
      dispSpeed(abs(ecu_data.speedKphRAW*0.621371));
    }
  } else if (nextPage==3) {
    if (ecu_data.engineRpmRAW != ecu_data_old.engineRpmRAW) {
      dispRev(ecu_data.engineRpmRAW);
    }
  } else if (nextPage==4) {
    if ((ecu_data.throttlePositionRAW != ecu_data_old.throttlePositionRAW)|(ecu_data.brakeOutputRAW != ecu_data_old.brakeOutputRAW)) {
      dispTps(ecu_data.throttlePositionRAW*100/1023);
      dispBrake(ecu_data.brakeOutputRAW*100/1023);
    }
  }
} //dispNext()
/**************************************************
 END FILE
**************************************************/

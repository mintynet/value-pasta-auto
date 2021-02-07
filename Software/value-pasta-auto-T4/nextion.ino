//**************************************************
// nextion_setup
//**************************************************

void nextion_setup() {
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
}

//**************************************************
// endNextion
//**************************************************

void endNextion () {
  NEXTION_PORT.write(0xff);
  NEXTION_PORT.write(0xff);
  NEXTION_PORT.write(0xff);
} //endNextion()

//**************************************************
// dispPowertrain
//**************************************************

void dispPowertrain () {
  int16_t tempVar = 0;
  tempVar = (int16_t)ecu_data.steeringValueRAW;
  if (tempVar < -8) {
    NEXTION_PORT.print(F("tLR.txt=\"L\""));
    endNextion();
    if (tempVar < -256) {
      NEXTION_PORT.print(F("pFL.pic=3"));
      endNextion();
      NEXTION_PORT.print(F("pFR.pic=3"));
      endNextion();
    } else {
      NEXTION_PORT.print(F("pFL.pic=4"));
      endNextion();
      NEXTION_PORT.print(F("pFR.pic=4"));
      endNextion();
    }
  } else if (tempVar < 8) {
    NEXTION_PORT.print(F("tLR.txt=\"S\""));
    endNextion();
    NEXTION_PORT.print(F("pFL.pic=5"));
    endNextion();
    NEXTION_PORT.print(F("pFR.pic=5"));
    endNextion();
  } else {
    NEXTION_PORT.print(F("tLR.txt=\"R\""));
    endNextion();
    if (tempVar > 256) {
      NEXTION_PORT.print(F("pFL.pic=7"));
      endNextion();
      NEXTION_PORT.print(F("pFR.pic=7"));
      endNextion();
    } else {
      NEXTION_PORT.print(F("pFL.pic=6"));
      endNextion();
      NEXTION_PORT.print(F("pFR.pic=6"));
      endNextion();
    }
  }
  tempVar = abs(tempVar);
  tempVar = tempVar*100/510;
  NEXTION_PORT.print(F("nLR.val="));
  NEXTION_PORT.print(tempVar);
  endNextion();
  tempVar = ecu_data.brakeValueRAW*100/1022;
  NEXTION_PORT.print(F("nBRAKE.val="));
  NEXTION_PORT.print(tempVar);
  endNextion();
  tempVar = ecu_data.acceleratorValueRAW*100/1022;
  NEXTION_PORT.print(F("nACC.val="));
  NEXTION_PORT.print(tempVar);
  endNextion();
  NEXTION_PORT.print(F("tDIAL.txt=\""));
  switch(dialCount) {
    case 1:
      NEXTION_PORT.print(F("S\""));
      break;
    case 2:
      NEXTION_PORT.print(F("R\""));
      break;
    case 3:
      NEXTION_PORT.print(F("T\""));
      break;
    case 4:
      NEXTION_PORT.print(F("F\""));
      break;
    case 6:
      NEXTION_PORT.print(F("*\""));
      break;
    case 7:
      NEXTION_PORT.print(F("B\""));
      break;
    default:      
      NEXTION_PORT.print(F("-\""));
      break;
  }
  endNextion();
  NEXTION_PORT.print(F("nPOT.val="));
  NEXTION_PORT.print(potValue);
  endNextion();
  switch(ecu_data.shiftPositionRAW) {
    case 1:
      if (ecu_data.parkingValueRAW == 0){
        NEXTION_PORT.print(F("t0.pco=65535"));
        endNextion();
        NEXTION_PORT.print(F("t0.bco=0"));
        endNextion();
      } else {
        NEXTION_PORT.print(F("t0.pco=65535"));
        endNextion();
        NEXTION_PORT.print(F("t0.bco=63488"));
        endNextion();
      }
      NEXTION_PORT.print(F("t1.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t1.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t2.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t2.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t3.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t3.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t4.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t4.bco=65535"));
      endNextion();
      break;
    case 2:
      NEXTION_PORT.print(F("t0.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t0.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t1.pco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t1.bco=0"));
      endNextion();
      NEXTION_PORT.print(F("t2.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t2.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t3.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t3.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t4.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t4.bco=65535"));
      endNextion();
      break;
    case 3:
      NEXTION_PORT.print(F("t0.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t0.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t1.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t1.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t2.pco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t2.bco=0"));
      endNextion();
      NEXTION_PORT.print(F("t3.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t3.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t4.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t4.bco=65535"));
      endNextion();
      break;
    case 4:
      NEXTION_PORT.print(F("t0.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t0.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t1.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t1.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t2.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t2.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t3.pco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t3.bco=0"));
      endNextion();
      NEXTION_PORT.print(F("t4.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t4.bco=65535"));
      endNextion();
      break;
    case 5:
      NEXTION_PORT.print(F("t0.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t0.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t1.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t1.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t2.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t2.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t3.pco=0"));
      endNextion();
      NEXTION_PORT.print(F("t3.bco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t4.pco=65535"));
      endNextion();
      NEXTION_PORT.print(F("t4.bco=0"));
      endNextion();
      break;
    default:
      //ecu_data.shiftPositionVal = "?";
    break; 
  }
} //dispChassis()

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
// dispBody
//**************************************************

void dispBody () {
  // MAIN FLASH
  if (ecu_data.lightFlValueRAW) {
    NEXTION_PORT.print(F("vis lMAIN,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rMAIN,1"));
    endNextion();
  } else if (ecu_data.lightIndicatorRAW == 0x01) {
    // SIDE LIGHTS
    NEXTION_PORT.print(F("vis lSIDE,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rSIDE,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lDIP,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rDIP,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lMAIN,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rMAIN,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis pPLATE,1"));
    endNextion();
  } else if (ecu_data.lightIndicatorRAW == 0x02) {
    // DIPPED LIGHT
    NEXTION_PORT.print(F("vis lSIDE,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rSIDE,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lDIP,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rDIP,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lMAIN,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rMAIN,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis pPLATE,1"));
    endNextion();
  } else if (ecu_data.lightIndicatorRAW == 0x03) {
    // SIDE AND DIPPED LIGHT
    NEXTION_PORT.print(F("vis lSIDE,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rSIDE,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lDIP,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rDIP,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lMAIN,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rMAIN,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis pPLATE,1"));
    endNextion();
  } else if (ecu_data.lightIndicatorRAW == 0x04) {
    // MAIN LIGHT
    NEXTION_PORT.print(F("vis lSIDE,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rSIDE,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lDIP,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rDIP,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lMAIN,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rMAIN,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis pPLATE,1"));
    endNextion();
  } else if (ecu_data.lightIndicatorRAW == 0x05) {
    // SIDE AND MAIN LIGHT
    NEXTION_PORT.print(F("vis lSIDE,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rSIDE,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lDIP,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rDIP,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lMAIN,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rMAIN,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis pPLATE,1"));
    endNextion();
  } else if (ecu_data.lightIndicatorRAW == 0x06) {
    // DIPPED AND MAIN LIGHT
    NEXTION_PORT.print(F("vis lSIDE,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rSIDE,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lDIP,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rDIP,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lMAIN,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rMAIN,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis pPLATE,1"));
    endNextion();
  } else if (ecu_data.lightIndicatorRAW == 0x07) {
    // SIDE, DIPPED AND MAIN LIGHT
    NEXTION_PORT.print(F("vis lSIDE,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rSIDE,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lDIP,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rDIP,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lMAIN,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rMAIN,1"));
    endNextion();
    NEXTION_PORT.print(F("vis lTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rTAIL,1"));
    endNextion();
    NEXTION_PORT.print(F("vis pPLATE,1"));
    endNextion();
  } else {
    // NO LIGHTS
    NEXTION_PORT.print(F("vis lSIDE,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rSIDE,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lDIP,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rDIP,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lMAIN,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rMAIN,0"));
    endNextion();
    NEXTION_PORT.print(F("vis lTAIL,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rTAIL,0"));
    endNextion();
    NEXTION_PORT.print(F("vis pPLATE,0"));
    endNextion();
  }
  // FRONT WIPER
  if (ecu_data.frontWiperStatusRAW == 0) {
    NEXTION_PORT.print(F("vF.val=0"));
    endNextion();
  } else if ((ecu_data.frontWiperStatusRAW & 0x08) >> 3) {
    // MIST
    NEXTION_PORT.print(F("tmF.tim=50"));
    endNextion();
    NEXTION_PORT.print(F("vF.val=1"));
    endNextion();
  } else if ((ecu_data.frontWiperStatusRAW & 0x04) >> 2) {
    // HIGH
    NEXTION_PORT.print(F("tmF.tim=250"));
    endNextion();
    NEXTION_PORT.print(F("vF.val=1"));
    endNextion();
  } else if ((ecu_data.frontWiperStatusRAW & 0x02) >> 1) {
    // LOW
    NEXTION_PORT.print(F("tmF.tim=750"));
    endNextion();
    NEXTION_PORT.print(F("vF.val=1"));
    endNextion();
  } else {
    // INT
    NEXTION_PORT.print(F("tmF.tim=1000"));
    endNextion();
    NEXTION_PORT.print(F("vF.val=1"));
    endNextion();
  }
  // REAR WIPER
  if (ecu_data.rearWiperStatusRAW == 0) {
    NEXTION_PORT.print(F("vR.val=0"));
    endNextion();
  } else if (ecu_data.rearWiperStatusRAW == 8) {
    // MIST
    NEXTION_PORT.print(F("tmR.tim=50"));
    endNextion();
    NEXTION_PORT.print(F("vR.val=1"));
    endNextion();
  } else {
    // LOW
    NEXTION_PORT.print(F("tmR.tim=750"));
    endNextion();
    NEXTION_PORT.print(F("vR.val=1"));
    endNextion();
  }
  // HAZARD LIGHTS
  if ((ecu_data_old.turnSignalIndicatorRAW == 3)|(ecu_data.turnSignalIndicatorRAW != 3)) {
    NEXTION_PORT.print(F("flashL.val=0"));
    endNextion();
    NEXTION_PORT.print(F("flashR.val=0"));
    endNextion();
  }
  // INDICATORS
  if (ecu_data.turnSignalIndicatorRAW == 3) {
    // HAZARD
    NEXTION_PORT.print(F("flashL.val=1"));
    endNextion();
    NEXTION_PORT.print(F("flashR.val=1"));
    endNextion();
  } else if (ecu_data.turnSignalIndicatorRAW == 1) {
    // LEFT
    NEXTION_PORT.print(F("flashL.val=1"));
    endNextion();
    NEXTION_PORT.print(F("flashR.val=0"));
    endNextion();
  } else if (ecu_data.turnSignalIndicatorRAW == 2) {
    // RIGHT
    NEXTION_PORT.print(F("flashR.val=0"));
    endNextion();
    NEXTION_PORT.print(F("flashR.val=1"));
    endNextion();
  } else {
    // NO INDICATORS
    NEXTION_PORT.print(F("flashL.val=0"));
    endNextion();
    NEXTION_PORT.print(F("flashR.val=0"));
    endNextion();
  }
  // LEFT WINDOW
  if (ecu_data.lDoorSwValueRAW == 2) {
    // DOWN
    NEXTION_PORT.print(F("vLEFT.val=vLEFT.val+3"));
    endNextion();
  } else if (ecu_data.lDoorSwValueRAW == 1) {
    // UP
    NEXTION_PORT.print(F("vLEFT.val=vLEFT.val+2"));
    endNextion();
  }
  // RIGHT WINDOW
  if (ecu_data.rDoorSwValueRAW == 2) {
    // DOWN
    NEXTION_PORT.print(F("vRIGHT.val=vRIGHT.val+3"));
    endNextion();
  } else if (ecu_data.rDoorSwValueRAW == 1) {
    // UP
    NEXTION_PORT.print(F("vRIGHT.val=vRIGHT.val+2"));
    endNextion();
  }
  // DOOR LOCK
  if ((ecu_data.doorLockStatusRAW & 0x03) == 3) {
    // BOTH LOCKED
    NEXTION_PORT.print(F("vis lLOCK,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rLOCK,1"));
    endNextion();
  } else if ((ecu_data.doorLockStatusRAW & 0x02) == 2) {
    // RIGHT LOCKED
    NEXTION_PORT.print(F("vis lLOCK,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rLOCK,1"));
    endNextion();
  } else if ((ecu_data.doorLockStatusRAW & 0x01) == 1) {
    // LEFT LOCKED
    NEXTION_PORT.print(F("vis lLOCK,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rLOCK,0"));
    endNextion();
  } else if ((ecu_data.doorLockStatusRAW & 0x03) == 0) {
    // UNLOCKED
    NEXTION_PORT.print(F("vis lLOCK,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rLOCK,0"));
    endNextion();
  }
  if (ecu_data.brakeValueRAW > 64){
    NEXTION_PORT.print(F("vis lSTOP,1"));
    endNextion();
    NEXTION_PORT.print(F("vis rSTOP,1"));
    endNextion();
  } else {
    NEXTION_PORT.print(F("vis lSTOP,0"));
    endNextion();
    NEXTION_PORT.print(F("vis rSTOP,0"));
    endNextion();
  }
} //dispBody()

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
  if(ecuNumber==0) {
    if((next_resp[0]==0x66) && (next_resp[1]==0x0)) 
    {
      nextPage=0;
      if(gDebug) DEBUG_PORT.println(F("CHANGE PAGE 0"));
    } else if((next_resp[0]==0x66) && (next_resp[1]==0x1)) 
    {
      nextPage=1;
      dispPowertrain();
      if(gDebug) DEBUG_PORT.println(F("CHANGE PAGE 1"));
    } 
  } else if (ecuNumber==1) {
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
  } else if (ecuNumber==2) {
    if((next_resp[0]==0x66) && (next_resp[1]==0x0)) 
    {
      nextPage=0;
      if(gDebug) DEBUG_PORT.println(F("CHANGE PAGE 0"));
    } else if((next_resp[0]==0x66) && (next_resp[1]==0x1)) 
    {
      nextPage=1;
      dispBody();
      if(gDebug) DEBUG_PORT.println(F("CHANGE PAGE 1"));
    } else if((next_resp[0]==0x65) && (next_resp[1]==0x1) && (next_resp[2]==0x25)) 
    {
      // BONNET
      ecu_data.bonnetOpenSwitchRAW = !ecu_data.bonnetOpenSwitchRAW;
      if(gDebug) DEBUG_PORT.println(F("BONNET"));
    } else if((next_resp[0]==0x65) && (next_resp[1]==0x1) && (next_resp[2]==0x26)) 
    {
      // BOOT
      ecu_data.trunkOpenSwitchRAW = !ecu_data.trunkOpenSwitchRAW;
      if(gDebug) DEBUG_PORT.println(F("BOOT"));
    } else if((next_resp[0]==0x65) && (next_resp[1]==0x1) && (next_resp[2]==0x27)) 
    {
      // LDOOR
      boolean lDoor = !((ecu_data.doorLockStatusRAW & 0x04) >> 2);
      ecu_data.doorLockStatusRAW = (ecu_data.doorLockStatusRAW & 0x0B)|(lDoor << 2);
      if ((ecu_data.doorLockStatusRAW & 0x01) >> 0) {
        ecu_data.doorLockStatusRAW = ecu_data.doorLockStatusRAW - 1;
      }
      if(gDebug) DEBUG_PORT.println(F("LDOOR"));
    } else if((next_resp[0]==0x65) && (next_resp[1]==0x1) && (next_resp[2]==0x28)) 
    {
      // RDOOR
      boolean rDoor = !((ecu_data.doorLockStatusRAW & 0x08) >> 3);
      ecu_data.doorLockStatusRAW = (ecu_data.doorLockStatusRAW & 0x07)|(rDoor << 3);
      if ((ecu_data.doorLockStatusRAW & 0x02) >> 1) {
        ecu_data.doorLockStatusRAW = ecu_data.doorLockStatusRAW - 2;
      }
      if(gDebug) DEBUG_PORT.println(F("RDOOR"));
    } else if (next_resp[0]<=20) {
        ecu_data.lDoorLimitRAW = next_resp[0]*5;
    } else if (next_resp[0]<=40) {
        ecu_data.rDoorLimitRAW = (next_resp[0]-20)*5;
    }
  }
} //checkNext

//**************************************************
// dispNext
//**************************************************

void dispNext() {
  if (ecuNumber==0) {
    if (nextPage==1) {
      if ((ecu_data.steeringValueRAW != ecu_data_old.steeringValueRAW)|
          (ecu_data.brakeValueRAW != ecu_data_old.brakeValueRAW)|
          (ecu_data.acceleratorValueRAW != ecu_data_old.acceleratorValueRAW)|
          (ecu_data.shiftPositionRAW != ecu_data_old.shiftPositionRAW)|
          (ecu_data.parkingValueRAW!=ecu_data_old.parkingValueRAW)|
          (ecu_data.mcpA!=ecu_data_old.mcpA)|(potValue!=potValue_old)) {
        dispPowertrain();
      }
    }
  } else if (ecuNumber==1) {
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
  } else if (ecuNumber==2) {
    if (nextPage==1) {
      if ((ecu_data.frontWiperStatusRAW!=ecu_data_old.frontWiperStatusRAW)|
          (ecu_data.rearWiperStatusRAW!=ecu_data_old.rearWiperStatusRAW)|
          (ecu_data.lightIndicatorRAW!=ecu_data_old.lightIndicatorRAW)|
          (ecu_data.turnSignalIndicatorRAW!=ecu_data_old.turnSignalIndicatorRAW)|
          (ecu_data.lDoorSwValueRAW!=ecu_data_old.lDoorSwValueRAW)|
          (ecu_data.rDoorSwValueRAW!=ecu_data_old.rDoorSwValueRAW)|
          (ecu_data.doorLockStatusRAW!=ecu_data_old.doorLockStatusRAW)|
          (ecu_data.brakeValueRAW!=ecu_data_old.brakeValueRAW)) {
        dispBody();
        if (gDebug) {
          DEBUG_PORT.println(F("************************************************************"));
          DEBUG_PORT.println(F("FRONT WIPER"));
          DEBUG_PORT.println(ecu_data.frontWiperStatusRAW,HEX);
          DEBUG_PORT.println(ecu_data_old.frontWiperStatusRAW,HEX);
          DEBUG_PORT.println(F("REAR WIPER"));
          DEBUG_PORT.println(ecu_data.rearWiperStatusRAW,HEX);
          DEBUG_PORT.println(ecu_data_old.rearWiperStatusRAW,HEX);
          DEBUG_PORT.println(F("LIGHTS"));
          DEBUG_PORT.println(ecu_data.lightIndicatorRAW,HEX);
          DEBUG_PORT.println(ecu_data_old.lightIndicatorRAW,HEX);
          DEBUG_PORT.println(F("INDICATORS"));
          DEBUG_PORT.println(ecu_data.turnSignalIndicatorRAW,HEX);
          DEBUG_PORT.println(ecu_data_old.turnSignalIndicatorRAW,HEX);
          DEBUG_PORT.println(F("LEFT DOOR"));
          DEBUG_PORT.println(ecu_data.lDoorSwValueRAW,HEX);
          DEBUG_PORT.println(ecu_data_old.lDoorSwValueRAW,HEX);
          DEBUG_PORT.println(F("RIGHT DOOR"));
          DEBUG_PORT.println(ecu_data.rDoorSwValueRAW,HEX);
          DEBUG_PORT.println(ecu_data_old.rDoorSwValueRAW,HEX);
          DEBUG_PORT.println(F("LOCKS"));
          DEBUG_PORT.println(ecu_data.doorLockStatusRAW,HEX);
          DEBUG_PORT.println(ecu_data_old.doorLockStatusRAW,HEX);
          DEBUG_PORT.println(F("BRAKES"));
          DEBUG_PORT.println(ecu_data.brakeValueRAW,HEX);
          DEBUG_PORT.println(ecu_data_old.brakeValueRAW,HEX);
          DEBUG_PORT.println(F("************************************************************"));
          DEBUG_PORT.println(millis());
          DEBUG_PORT.println(F("************************************************************"));
        }
      }
    }
  }
} //dispNext()
/**************************************************
 END FILE
**************************************************/

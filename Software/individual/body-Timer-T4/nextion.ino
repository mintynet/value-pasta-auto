//**************************************************
// endNextion
//**************************************************

void endNextion () {
  NEXTION_PORT.write(0xff);
  NEXTION_PORT.write(0xff);
  NEXTION_PORT.write(0xff);
} //endNextion()

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
  if (NEXTION_PORT.available()>0)
  {
    if(gDebug) DEBUG_PORT.println(F("NEXT DATA"));
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
} //checkNext

//**************************************************
// dispNext
//**************************************************

void dispNext() {
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
} //dispNext()
/**************************************************
 END FILE
**************************************************/

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
    dispPowertrain();
    if(gDebug) DEBUG_PORT.println(F("CHANGE PAGE 1"));
  } 
} //checkNext

//**************************************************
// dispNext
//**************************************************

void dispNext() {
  if (nextPage==1) {
    if ((ecu_data.steeringValueRAW != ecu_data_old.steeringValueRAW)|(ecu_data.brakeValueRAW != ecu_data_old.brakeValueRAW)|(ecu_data.acceleratorValueRAW != ecu_data_old.acceleratorValueRAW)|(ecu_data.shiftPositionRAW != ecu_data_old.shiftPositionRAW)|(ecu_data.parkingValueRAW!=ecu_data_old.parkingValueRAW)|(ecu_data.mcpA!=ecu_data_old.mcpA)|(potValue!=potValue_old)) {
      dispPowertrain();
      if (gDebug == true) {
        DEBUG_PORT.println(F("STEER"));
        DEBUG_PORT.println(ecu_data.steeringValueRAW,HEX);
        DEBUG_PORT.println(ecu_data_old.steeringValueRAW,HEX);
        DEBUG_PORT.println(F("BRAKE"));
        DEBUG_PORT.println(ecu_data.brakeValueRAW,HEX);
        DEBUG_PORT.println(ecu_data_old.brakeValueRAW,HEX);
        DEBUG_PORT.println(F("ACCELERATOR"));
        DEBUG_PORT.println(ecu_data.acceleratorValueRAW,HEX);
        DEBUG_PORT.println(ecu_data_old.acceleratorValueRAW,HEX);
        DEBUG_PORT.println(F("SHIFT"));
        DEBUG_PORT.println(ecu_data.shiftPositionRAW,HEX);
        DEBUG_PORT.println(ecu_data_old.shiftPositionRAW,HEX);
        DEBUG_PORT.println(F("PARKING"));
        DEBUG_PORT.println(ecu_data.parkingValueRAW,HEX);
        DEBUG_PORT.println(ecu_data_old.parkingValueRAW,HEX);
        DEBUG_PORT.println(millis());
      }
    }
  }
} //dispNext()

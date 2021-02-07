#include "mcp_minty.h"

/**************************************************
 MCP_CAN_MINTY
**************************************************/
MCP_CAN_MINTY::MCP_CAN_MINTY(INT8U _CS, INT8U _TX0BUF)
{
    MCPCS = _CS;
    MCPTX0BUF = _TX0BUF;
    MCP2515_TX0BUF_UNSELECT();
    pinMode(MCPTX0BUF, OUTPUT);
} // 

/**************************************************
 setupTX0Buf()
**************************************************/

void MCP_CAN_MINTY::setupTX0Buf (const INT32U id, const INT8U len, const INT8U *data, bool fastMode)
{
  uint16_t canid;
  uint8_t tbufdata[4];
  canid = (uint16_t)(id & 0x0FFFF);
  
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  // LOAD TX BUFFER 0
  MCP2515_SELECT();
  SPI.transfer(MCP_LOAD_TX0);
  if ((0x80000000 & id)==0x80000000) {
    tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
    tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
    canid = (uint16_t)(id>>16);
    tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
    tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x0C) << 3);
    tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
    tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5);
    for (int i=0;i<4;i++){
      SPI.transfer(tbufdata[i]); 
    }    
  } else {
    SPI.transfer(canid>>3);
    SPI.transfer((canid & 0x07 ) << 5);
    SPI.transfer(0x0);
    SPI.transfer(0x0);
  }
  SPI.transfer(len);
  for (int i=0;i<len;i++) {
    SPI.transfer(data[i]);
  }
  MCP2515_UNSELECT();
  SPI.endTransaction();
  if (!fastMode) {
    delayMicroseconds(150);
  }
} // setupTX0Buf()

/**************************************************
 readCANStatus()
**************************************************/

INT8U MCP_CAN_MINTY::readCANStatus() // same as MCP_CAN::mcp2515_readStatus
{
  byte ret;
  
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_SELECT();
  SPI.transfer(MCP_READ_STATUS);
  ret = SPI.transfer(0x00);
  MCP2515_UNSELECT();
  SPI.endTransaction();

  return ret;
} // readCANStatus()

/**************************************************
 modifyRegister()
**************************************************/

void MCP_CAN_MINTY::modifyRegister(const uint8_t address, const uint8_t mask, const uint8_t data)
{
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_SELECT();
  SPI.transfer(MCP_BITMOD);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(data);
  MCP2515_UNSELECT();
  SPI.endTransaction();
} // modifyRegister()

/**************************************************
 tx0RTS0()
**************************************************/

INT8U MCP_CAN_MINTY::tx0RTS()
{
//  READY TO SEND
#if MCPADDPINS
//  Use TX0RTS pin
  MCP2515_TX0BUF_SELECT();
  MCP2515_TX0BUF_UNSELECT();
  //Serial.println("TX0BUF");
#else
//  Use SPI to set RTS
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_SELECT();
  SPI.transfer(MCP_RTS_TX0);
  MCP2515_UNSELECT();
  SPI.endTransaction();
  //Serial.println("SPI");
#endif
  delayMicroseconds(120);
  byte res;
  byte idx = 0;
  do {
    idx++;
    delayMicroseconds(20);
    res = readCANStatus();
    res = (res & 0x08)>>3;
  } while ((!res) && (idx<TIMEOUTVALUE));
  if (idx==TIMEOUTVALUE) {
    return CANSENDTIMEOUT;
  }
  modifyRegister(MCP_CANINTF,0x04,0x00);
  return CAN_OK;
} // tx0RTS()

/**************************************************
 sendTX0()
**************************************************/

INT8U MCP_CAN_MINTY::sendTX0(const INT32U id, const INT8U len, const INT8U *data, const bool fastMode)
{
  INT8U res;
  setupTX0Buf(id,len,data,fastMode);
  res = tx0RTS();
  return res;
} // sendTX0()

/**************************************************
 END FILE
**************************************************/

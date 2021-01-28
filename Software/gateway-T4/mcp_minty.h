#include <SPI.h>

#define CAN3_CS 10
#define CAN3_TX0BUF         24                // TX0 RTS Pin
#define CAN3_TX1BUF         25                // TX1 RTS Pin
#define CAN3_TX2BUF         26                // TX2 RTS Pin
#define CAN3_RX0BF          27                // RX0 INT Pin
#define CAN3_RX1BF          28                // RX1 INT Pin

#define TIMEOUTVALUE        50
#define CAN_OK              (0)
#define CANSENDTIMEOUT      (200)

#define MCP_SIDH            0x0
#define MCP_SIDL            0x1
#define MCP_EID8            0x2
#define MCP_EID0            0x3

#define MCP_TXB_EXIDE_M     0x08

#define MCP_LOAD_TX0        0x40
#define MCP_LOAD_TX1        0x42
#define MCP_LOAD_TX2        0x44

#define MCP_READ_STATUS     0xA0

#define MCP_RTS_TX0         0x81
#define MCP_RTS_TX1         0x82
#define MCP_RTS_TX2         0x84
#define MCP_RTS_ALL         0x87

//**************************************************
// setupTX0Buf()
//**************************************************

void setupTX0Buf (unsigned long id, byte len, uint8_t *data, bool fastMode)
{
  uint16_t canid;
  uint8_t tbufdata[4];
  canid = (uint16_t)(id & 0x0FFFF);
  
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  // LOAD TX BUFFER 0
  digitalWrite(CAN3_CS, LOW);
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
  digitalWrite(CAN3_CS, HIGH);
  SPI.endTransaction();
  if (!fastMode) {
    delayMicroseconds(150);
  }
} // setupTX0Buf()

//**************************************************
// readCANStatus()
//**************************************************

byte readCANStatus() // same as MCP_CAN::mcp2515_readStatus
{
  byte ret;
  
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN3_CS, LOW);
  SPI.transfer(MCP_READ_STATUS);
  ret = SPI.transfer(0x00);
  digitalWrite(CAN3_CS, HIGH);
  SPI.endTransaction();

  return ret;
} // readCANStatus()

//**************************************************
// tx0RTS0()
//**************************************************

byte tx0RTS()
{
  // READY TO SEND
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN3_CS, LOW);
  SPI.transfer(MCP_RTS_TX0);
  digitalWrite(CAN3_CS, HIGH);
  SPI.endTransaction();
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
  return CAN_OK;
} // tx0RTS()

//**************************************************
// sendTX0()
//**************************************************

byte sendTX0(unsigned long id, byte len, uint8_t *data, bool fastMode)
{
  byte res;
  setupTX0Buf(id,len,data,fastMode);
  res = tx0RTS();
  return res;
} // sendTX0()

/*
//**************************************************
// txAllRTS()
//**************************************************

byte txALLRTS()
{
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  // READY TO SEND
  digitalWrite(CAN3_CS, LOW);
  SPI.transfer(MCP_RTS_ALL);
  digitalWrite(CAN3_CS, HIGH);
  SPI.endTransaction();
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
  return CAN_OK;
} // txAllRTS() */

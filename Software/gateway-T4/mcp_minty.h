#include <SPI.h>

#define TIMEOUTVALUE   50
#define CAN_OK         (0)
#define CANSENDTIMEOUT (200)

#define MCP_LOAD_TX0        0x40
//#define MCP_LOAD_TX1        0x42
//#define MCP_LOAD_TX2        0x44

#define MCP_READ_STATUS     0xA0

#define MCP_RTS_TX0         0x81
//#define MCP_RTS_TX1         0x82
//#define MCP_RTS_TX2         0x84
//#define MCP_RTS_ALL         0x87

void sendStdTX0 (unsigned long id, byte len, uint8_t *data)
{
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  // LOAD TX BUFFER 0
  digitalWrite(10, LOW);
  SPI.transfer(MCP_LOAD_TX0);
  SPI.transfer(id>>3);
  SPI.transfer((id & 0x07 ) << 5);
  SPI.transfer(0x0);
  SPI.transfer(0x0);
  SPI.transfer(len);
  for (int i=0;i<len;i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(10, HIGH);
  delayMicroseconds(250);
  SPI.endTransaction();
}

byte readCANStatus()
{
  byte ret;
  
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(10, LOW);
  SPI.transfer(MCP_READ_STATUS);
  ret = SPI.transfer(0x00);
  digitalWrite(10, HIGH);
  SPI.endTransaction();

  return ret;
}

byte tx0RTS()
{
  // READY TO SEND
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(10, LOW);
  SPI.transfer(MCP_RTS_TX0);
  digitalWrite(10, HIGH);
  delayMicroseconds(250);
  SPI.endTransaction();
  byte res;
  byte idx;
  do {
    idx++;
    res = readCANStatus();
    res = (res & 0x08)>>3;
  } while ((!res) && (idx<TIMEOUTVALUE));
  if (idx==TIMEOUTVALUE) {
    Serial.println("TIMEOUT");
    return CANSENDTIMEOUT;
  }
  return CAN_OK;
}

/*void sendStdTX1 (unsigned long id, byte len, uint8_t *data)
{
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  // LOAD TX BUFFER 0
  digitalWrite(10, LOW);
  SPI.transfer(MCP_LOAD_TX1);
  SPI.transfer(id>>3);
  SPI.transfer((id & 0x07 ) << 5);
  SPI.transfer(0x0);
  SPI.transfer(0x0);
  SPI.transfer(len);
  for (int i=0;i<len;i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(10, HIGH);
  delayMicroseconds(250);
}

void sendStdTX2 (unsigned long id, byte len, uint8_t *data)
{
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  // LOAD TX BUFFER 0
  digitalWrite(10, LOW);
  SPI.transfer(MCP_LOAD_TX2);
  SPI.transfer(id>>3);
  SPI.transfer((id & 0x07 ) << 5);
  SPI.transfer(0x0);
  SPI.transfer(0x0);
  SPI.transfer(len);
  for (int i=0;i<len;i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(10, HIGH);
  delayMicroseconds(250);
  // READY TO SEND
  digitalWrite(10, LOW);
  SPI.transfer(MCP_RTS_TX2);
  digitalWrite(10, HIGH);
  delayMicroseconds(250);
  SPI.endTransaction();
}

void tx1RTS()
{
  // READY TO SEND
  digitalWrite(10, LOW);
  SPI.transfer(MCP_RTS_TX1);
  digitalWrite(10, HIGH);
  delayMicroseconds(250);
  SPI.endTransaction();
}

void tx2RTS()
{
  // READY TO SEND
  digitalWrite(10, LOW);
  SPI.transfer(MCP_RTS_TX2);
  digitalWrite(10, HIGH);
  delayMicroseconds(250);
  SPI.endTransaction();
}

void txALLRTS()
{
  // READY TO SEND
  digitalWrite(10, LOW);
  SPI.transfer(MCP_RTS_ALL);
  digitalWrite(10, HIGH);
  delayMicroseconds(250);
  SPI.endTransaction();
}*/

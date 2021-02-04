// ************************************************
// To use TXnBUF pins edit mcp_can.cpp
// change mcp2515_setRegister(MCP_TXRTSCTRL,0x00);
// to     mcp2515_setRegister(MCP_TXRTSCTRL,0x07);
// ************************************************
#ifndef _MCP_MINTY_H
#define _MCP_MINTY_H

#include <SPI.h>
#include "mcp_can_dfs.h"

#define             MCPADDPINS  1                 // TXnBUF and RXnBF pins present

#define MCP2515_TX0BUF_SELECT()   digitalWrite(MCPTX0BUF, LOW)
#define MCP2515_TX0BUF_UNSELECT() digitalWrite(MCPTX0BUF, HIGH)

class MCP_CAN_MINTY
{
  private:
  INT8U   MCPCS;
  INT8U   MCPTX0BUF;

  private:
  void setupTX0Buf (const INT32U id, const INT8U len, const INT8U *data, const bool fastMode);
  void modifyRegister(const INT8U address, const INT8U mask, const INT8U data);
  INT8U readCANStatus(void);
  INT8U tx0RTS(void);

  public:
  MCP_CAN_MINTY(INT8U _CS, INT8U _TX0BUF);
  byte sendTX0(INT32U id, INT8U len, INT8U *data, bool fastMode);
};

#endif
/**************************************************
 END FILE
**************************************************/

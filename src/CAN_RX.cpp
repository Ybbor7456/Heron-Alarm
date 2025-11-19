#include <SPI.h>
#include "mcp_can.h"

constexpr byte CS_PIN   = 17;   // MCP2155 CS
constexpr byte INT_PIN  = 20;   // only necessary if the board lacks a pull-up
constexpr byte LED_PIN  = 8;
constexpr unsigned CAN_ID_MATCH = 0x101;

MCP_CAN CAN(CS_PIN);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(INT_PIN, INPUT_PULLUP);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init FAIL"); while (1) {}
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("RX ready");
}

void loop() {
  if (CAN.checkReceive() == CAN_MSGAVAIL /*CAN_NOMSG*/) {
    unsigned long id = 0;
    byte len = 0;
    byte buf[8];

    if (CAN.readMsgBuf(&id, &len, buf) == CAN_OK) {
     /* Serial.print("RX id=0x"); Serial.print(id, HEX);
      Serial.print(" len="); Serial.print(len);
      Serial.print(" data:"); */
      if (id == CAN_ID_MATCH && len >= 1) {
        digitalWrite(LED_PIN, buf[0] == 0x01 ? HIGH : LOW);
        Serial.println(buf[0] == 0x01 ? "LED ON (motion)" : "LED OFF (clear)");
      }
    } else {
      Serial.println("readMsgBuf failed");
    }
  }
}

#include <SPI.h>
#include "mcp_can.h"

constexpr byte INT_PIN   = 20;    // MCP2515 INT (active-low)
constexpr byte SCK_PIN   = 18;    // SPI0 defaults
constexpr byte MOSI_PIN  = 19;
constexpr byte MISO_PIN  = 16;
constexpr byte CS_PIN    = 17;

constexpr uint8_t LED_PIN = 8;    // LED on GP8 with series resistor
constexpr unsigned CAN_ID = 0x101;

MCP_CAN CAN(CS_PIN);

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(INT_PIN, INPUT);        // interrupt line from MCP2515

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init FAIL"); while(1){}
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("RX ready");
}

void loop() {
  // Either poll INT or call CAN.checkReceive()
  if (digitalRead(INT_PIN) == LOW) {
    long unsigned id = 0;
    byte len = 0, buf[8] = {0};
    CAN.readMsgBuf(&id, &len, buf);

    if (id == CAN_ID && len >= 1) {
      if (buf[0] == 0x01) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED ON (motion)");
      } else {
        digitalWrite(LED_PIN, LOW);
        Serial.println("LED OFF (clear)");
      }
    }
  }
}

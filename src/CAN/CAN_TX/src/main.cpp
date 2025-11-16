#include <SPI.h>
#include "mcp_can.h"

// constexpr byte INT_PIN   = 20;    // unused but wired
constexpr byte SCK_PIN   = 18;    // SPI defaults
constexpr byte MOSI_PIN  = 19;    // pico default mosi pin
constexpr byte MISO_PIN  = 16;    // pico default miso pin 
constexpr byte CS_PIN    = 17;

constexpr uint8_t SENS_PIN = 8;  /// interchangeable between tx pico and rx pic
constexpr unsigned CAN_ID  = 0x101;

MCP_CAN CAN(CS_PIN);

bool pirPrev = false;


// send message if on 
void sendMotion(bool on) {
  byte b = on ? 0x01 : 0x00;
  CAN.sendMsgBuf(CAN_ID, 0, 1, &b);
}

void setup() {
  Serial.begin(115200);

  pinMode(SENS_PIN, INPUT_PULLDOWN); 

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init FAIL"); while(1){}
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("TX ready");
}
// add dbnce 
void loop() {
  bool pir = digitalRead(SENS_PIN);

  if (pir && !pirPrev) { // check states
    pirPrev = true;
    sendMotion(true);
    Serial.println("Motion On");
  }

  if (!pir && pirPrev) {
    pirPrev = false;
    sendMotion(false);
    Serial.println("Motion off");
  }
}

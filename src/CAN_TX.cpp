#include <SPI.h>
#include "mcp_can.h"

constexpr byte INT_PIN   = 20;    // not used actively here, but wired
constexpr byte SCK_PIN   = 18;    // SPI0 defaults
constexpr byte MOSI_PIN  = 19;
constexpr byte MISO_PIN  = 16;
constexpr byte CS_PIN    = 17;

constexpr uint8_t SENS_PIN = 8;   // PIR OUT pin (to Pico)
constexpr unsigned CAN_ID  = 0x101;

MCP_CAN CAN(CS_PIN);

bool pirPrev = false;
uint32_t lastChangeMs = 0;
const uint32_t debounceMs = 80;   // tame chatter

void sendMotion(bool on) {
  byte b = on ? 0x01 : 0x00;
  CAN.sendMsgBuf(CAN_ID, 0, 1, &b);
}

void setup() {
  Serial.begin(115200);

  pinMode(SENS_PIN, INPUT_PULLDOWN);   // important

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init FAIL"); while(1){}
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("TX ready");
}

void loop() {
  bool pir = digitalRead(SENS_PIN);
  uint32_t now = millis();

  // Debounced rising edge (OFF->ON): send ON
  if (pir && !pirPrev && (now - lastChangeMs > debounceMs)) {
    lastChangeMs = now;
    pirPrev = true;
    sendMotion(true);
    Serial.println("Motion ON");
  }

  // Debounced falling edge (ON->OFF): send OFF
  if (!pir && pirPrev && (now - lastChangeMs > debounceMs)) {
    lastChangeMs = now;
    pirPrev = false;
    sendMotion(false);
    Serial.println("Motion OFF");
  }
}

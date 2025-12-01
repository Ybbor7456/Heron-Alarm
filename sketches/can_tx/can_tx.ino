#include <SPI.h>
#include "mcp_can.h"              // https://github.com/coryjfowler/MCP_CAN_lib
                                  // https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf
constexpr byte INT_PIN   = 20;    // not used actively here (used in rx), but wired
constexpr byte SCK_PIN   = 18;    // SPI0 defaults
constexpr byte MOSI_PIN  = 19;
constexpr byte MISO_PIN  = 16;
constexpr byte CS_PIN    = 17;
constexpr uint8_t TEST_LED = 9; 
constexpr uint8_t SENS_PIN = 8;   // PIR OUT pin
constexpr unsigned CAN_ID  = 0x101; // use same for rx

MCP_CAN CAN(CS_PIN);
bool pirPrev = false;
uint32_t lastChangeMs = 0;
const uint32_t debounceMs = 80;  

bool sendMotion(bool on) {
  byte b = on ? 0x01 : 0x00;
  byte rc = CAN.sendMsgBuf(0x101, 0, 1, &b);   
  /*
  Serial.print("TX "); 
  Serial.print(on ? "ON" : "OFF");    
  Serial.print(" rc="); 
  Serial.println(rc == CAN_OK ? "OK" : "FAIL"); */
  return rc == CAN_OK;
}

void setup() {
  Serial.begin(115200);
  // wait up to ~3 s for the USB CDC to enumerate (only if USB is connected)
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { /* wait */ }
  delay(100);  // small extra cushion

  pinMode(SENS_PIN, INPUT_PULLDOWN);
  pinMode(TEST_LED, OUTPUT);
  digitalWrite(TEST_LED, LOW);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) { // same KBPS and MHZ on rx
    Serial.println("CAN init FAIL"); while (1) {}
  }
  //CAN.setMode(MCP_LOOPBACK); 
  CAN.setMode(MCP_NORMAL);
  Serial.println("TX ready");
}

void loop() {
  bool pir = digitalRead(SENS_PIN); // 0/1
  uint32_t now = millis();

  // debounce rising
  if (pir && !pirPrev && (now - lastChangeMs > debounceMs)) {
    lastChangeMs = now;
    pirPrev = true;
    digitalWrite(TEST_LED, HIGH);
    sendMotion(true);
    Serial.println("Motion ON");
  }
  // debounce falling
  if (!pir && pirPrev && (now - lastChangeMs > debounceMs)) {
    lastChangeMs = now;
    pirPrev = false;
    digitalWrite(TEST_LED, LOW);
    sendMotion(false);
    Serial.println("Motion OFF");
  }
}

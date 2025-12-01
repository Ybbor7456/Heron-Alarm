#include <Arduino.h>

constexpr uint8_t PIN_DE_RE = 2;
constexpr uint32_t BAUD     = 38400;

constexpr uint8_t LED_PIN   = 16;
constexpr uint16_t FLASH_MS = 2000;   // flash length

static inline uint8_t crc8(uint8_t x){ return x ^ 0xA5; }

void setup(){
  //while(!Serial) {}

  Serial1.setFIFOSize(256);
  Serial1.begin(BAUD);

  pinMode(PIN_DE_RE, OUTPUT);
  digitalWrite(PIN_DE_RE, LOW);      // always in receive

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  //Serial.println("[Receiver] Listening for EVT=0x01…");
}

void loop(){
  static enum { IDLE, GOT_AA, GOT_EVT } st = IDLE;
  static uint8_t evt = 0;
  Serial.println("hi");
  while (Serial1.available()){ 
    uint8_t b = Serial1.read();
    switch (st){
      case IDLE:    st = (b == 0xAA) ? GOT_AA : IDLE;           break;
      case GOT_AA:  evt = b;            st = GOT_EVT;           break;
      case GOT_EVT: {
        uint8_t c = b;
        if (c == crc8(evt)) {
          if (evt == 0x01) {
            // flash LED
            digitalWrite(LED_PIN, HIGH);
            delay(FLASH_MS);
            digitalWrite(LED_PIN, LOW);
          //Serial.println("[Receiver] Trigger received → LED flash");
          } else {
          //Serial.print("[Receiver] Unknown EVT 0x"); Serial.println(evt, HEX);
          }
        } else {
        //Serial.println("[Receiver] CRC FAIL (frame ignored)");
        }
        st = IDLE;
      } break;
    }
  }
}
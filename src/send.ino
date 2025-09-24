// ==== Pico #1: ONE sensor -> RS-485 ====
// Wire: Sensor OUT -> GP6 (with INPUT_PULLDOWN); GND common

#include <Arduino.h>

constexpr uint8_t PIN_DE_RE = 2;   // tie DE & /RE here
constexpr uint32_t BAUD     = 38400;

constexpr uint8_t SENS_PIN  = 8;   // GP8: your single sensor
// ---------- tiny CRC-8 (ATM) for 1-byte payload ----------
static inline uint8_t crc8(uint8_t x) { return x ^ 0xA5; }  // keep same on RX

// RS-485 transmit helper
static void rs485_send(const uint8_t* p, size_t n) {
  digitalWrite(PIN_DE_RE, HIGH);         // TX enable (drive bus)
  Serial1.write(p, n);
  Serial1.flush();                        // wait until sent
  delayMicroseconds(200);                 // a few char times
  digitalWrite(PIN_DE_RE, LOW);           // back to RX mode
}

void setup() {
  // UART0 on GP0/GP1 is Serial1 in Earle's Arduino-Pico core
  Serial1.setFIFOSize(256);
  Serial1.begin(BAUD);

  pinMode(PIN_DE_RE, OUTPUT);
  digitalWrite(PIN_DE_RE, LOW);           // start in receive mode

  pinMode(SENS_PIN, INPUT_PULLDOWN);      // stable default LOW
}

void loop() {
  static uint8_t last = 0xFF;

  uint8_t mask = digitalRead(SENS_PIN) ? 0x01 : 0x00; // only bit0 used

  if (mask != last) {                      // send only on change
    last = mask;
    uint8_t frame[3] = {0xAA, mask, crc8(mask)}; // [start][payload][crc]
    rs485_send(frame, sizeof(frame));
  }
  delay(10);
}

// ==== Pico #2: RS-485 -> ONE LED ====
// Wire: LED + resistor (~330Ω) from GP16 to GND

#include <Arduino.h>

constexpr uint8_t PIN_DE_RE = 2;   // tie DE & /RE here (LOW = listen)
constexpr uint32_t BAUD     = 38400;

constexpr uint8_t LED_PIN   = 16;  // GP16 drives the single LED

static inline uint8_t crc8(uint8_t x) { return x ^ 0xA5; } // must match TX

void setup() {
  Serial1.setFIFOSize(256);
  Serial1.begin(BAUD);

  pinMode(PIN_DE_RE, OUTPUT);
  digitalWrite(PIN_DE_RE, LOW);     // receive mode

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Parse fixed 3-byte frame: 0xAA, mask, crc
  enum { IDLE, GOT_AA, GOT_MASK } st = IDLE;
  uint8_t mask = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    switch (st) {
      case IDLE:
        st = (b == 0xAA) ? GOT_AA : IDLE;
        break;

      case GOT_AA:
        mask = b;
        st = GOT_MASK;
        break;

      case GOT_MASK: {
        uint8_t c = b;
        if (c == crc8(mask)) {
          // Only bit0 matters; HIGH = LED on
          digitalWrite(LED_PIN, (mask & 0x01) ? HIGH : LOW);
        }
        st = IDLE;
      } break;
    }
  }
}

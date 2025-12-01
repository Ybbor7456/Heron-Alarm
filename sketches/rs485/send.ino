#include <Arduino.h>

constexpr uint8_t PIN_DE_RE   = 2;     // DE & /RE tied here (LOW=RX, HIGH=TX)
constexpr uint32_t BAUD       = 38400;

constexpr uint8_t SENS_PIN    = 8;     // GP8 is your digital sensor input
constexpr uint16_t DEBOUNCE_MS= 50;    // debounce for noisy sensors

// Frame format: [0xAA][EVT][CRC8], where EVT=0x01 means "trigger"
static inline uint8_t crc8(uint8_t x) { return x ^ 0xA5; } //XOR

static void rs485_send_evt(uint8_t evt) {
  digitalWrite(PIN_DE_RE, HIGH);          // TX
  uint8_t frame[3] = {0xAA, evt, crc8(evt)};  //
  Serial1.write(frame, sizeof(frame));
  Serial1.flush(); 
  delayMicroseconds(200);
  digitalWrite(PIN_DE_RE, LOW);           // back to RX
}

void setup() {
  // USB debug
  Serial.begin(115200);
  //while (!Serial) {}

  // RS-485 UART (UART0 = Serial1 on GP0/GP1)
  Serial1.setFIFOSize(256);
  Serial1.begin(BAUD);

  pinMode(PIN_DE_RE, OUTPUT);
  digitalWrite(PIN_DE_RE, LOW);           // listen by default

  pinMode(SENS_PIN, INPUT_PULLDOWN);      // default LOW
  Serial.println("[Sender] Ready. Waiting for sensor rising edge…");
}

void loop() {
  static bool last = false;
  static uint32_t t_last_change = 0;

  bool now = digitalRead(SENS_PIN);
  uint32_t t = millis();

  // simple debounce: only react if stable for DEBOUNCE_MS
  if (now != last && (t - t_last_change) >= DEBOUNCE_MS) {
    t_last_change = t;
    last = now;

    if (now) { // rising edge = trigger event
      rs485_send_evt(0x01);
      Serial.println("[Sender] TRIGGER sent (EVT=0x01)");
    }
  }
}
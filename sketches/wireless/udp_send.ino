#include <WiFi.h>
#include <WiFiUdp.h>

/*
const char* SSID = "wifiname";
const char* PASS = "wifipassword";
*/

const char* SSID = "";
const char* PASS = "";

// Set receiver’s IP
IPAddress RX_IP(192,168,1,207);

constexpr int UDP_PORT = 5005;

constexpr int  SENS_PIN = 8;
constexpr int DEBOUNCE_MS = 50;

WiFiUDP udp;

static inline uint8_t crc(uint8_t x){ return x ^ 0xA5; }

void setup() {
  Serial.begin(115200);
  pinMode(SENS_PIN, INPUT_PULLDOWN);

  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.print("\nIP: "); Serial.println(WiFi.localIP());
}

void loop() {
  static bool last = false; 
  static uint32_t tLast = 0;

  bool now = digitalRead(SENS_PIN);
  uint32_t t = millis();

  if (now != last && (t - tLast) >= DEBOUNCE_MS) {
    tLast = t; last = now;
    if (now) {
      uint8_t evt = 0x01;
      uint8_t frame[3] = {0xAA, evt, crc(evt)};
      udp.beginPacket(RX_IP, UDP_PORT);
      udp.write(frame, sizeof(frame));
      udp.endPacket();
      Serial.println("TRIGGER sent");
    }
  }
}

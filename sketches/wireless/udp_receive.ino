#include <WiFi.h>
#include <WiFiUdp.h>
/*
const char* SSID = "wifiname";
const char* PASS = "password";
*/

const char* SSID = "ATTv5EQE6a";
const char* PASS = "3k4u=+hztju+";

constexpr int UDP_PORT = 5005;
constexpr int  LED_PIN  = 16;
constexpr int HOLD_MS  = 800;

WiFiUDP udp;
int ledOffAt = 0;

static inline uint8_t crc(uint8_t x) { return x ^ 0xA5; }



void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  WiFi.begin(SSID, PASS);

  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.print("\nIP: "); Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
  Serial.print("UDP listening on "); Serial.println(UDP_PORT);
} 


void loop() {
  // Non-blocking LED hold-off
  if (digitalRead(LED_PIN) && (int32_t)(millis() - ledOffAt) >= 0) {
    digitalWrite(LED_PIN, LOW);
  }

  int n = udp.parsePacket();
  if (n >= 3) {
    uint8_t buf[8];
    n = udp.read(buf, sizeof(buf));
    // Expect [0xAA][EVT][CHK]
    if (n >= 3 && buf[0] == 0xAA) {
      uint8_t evt = buf[1], c = buf[2];
      if (c == crc(evt) && evt == 0x01) { // crc check for corrupted bits
        digitalWrite(LED_PIN, HIGH);
        ledOffAt = millis() + HOLD_MS;
      }
    }
  }
}

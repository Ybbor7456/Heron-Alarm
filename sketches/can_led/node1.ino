#include <SPI.h>
#include "mcp_can.h"              // https://github.com/coryjfowler/MCP_CAN_lib
#include <Wire.h>                 //TWI library      wire.begin(), wire.Setclock(), wire.Wrte, ect   
#include <Adafruit_GFX.h>       
#include <Adafruit_SSD1306.h> 
                                  // https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf
constexpr byte INT_PIN   = 20;    // not used actively here (used in rx), but wired
constexpr byte SCK_PIN   = 18;    // SPI0 defaults
constexpr byte MOSI_PIN  = 19;
constexpr byte MISO_PIN  = 16;
constexpr byte CS_PIN    = 17;
constexpr uint8_t LED_PIN = 9; 
// constexpr uint8_t SENS_PIN = 8;   // PIR OUT pin -> temp removed and replaced with potentiometer
constexpr uint8_t POT_PIN = A0; 
//constexpr unsigned CAN_ID  = 0x101; // use same for rx
constexpr unsigned ID_NODE1 = 0x310;   // Pi #1 (this board)
constexpr unsigned ID_NODE2 = 0x311;   // Pi #2
constexpr unsigned ID_NODE3 = 0x312;   // Pi #3

constexpr byte LED_OFF = 0x00;
constexpr byte LED_ON  = 0x01;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define DRAW_H 20
#define DRAW_W 88
#define OLED_RESET -1           // No reset pin on many I2C modules; share MCU reset
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const int   ADCMAX = 4095;
MCP_CAN CAN(CS_PIN);

void drawOLED(uint16_t raw) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print(F("ADC: ")); display.print(raw);

  int w = (int)((uint32_t)raw * (SCREEN_WIDTH - 1) / 4095);
  display.drawRect(0, 30, SCREEN_WIDTH, 14, SSD1306_WHITE);
  display.fillRect(0, 30, w, 14, SSD1306_WHITE);
  display.display();
}

static uint16_t avg_adc(uint8_t n = 8, int pin = POT_PIN) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < n; ++i) {
    sum += analogRead(pin);        // 0..4095
  }
  return (uint16_t)(sum / n);
}

static int band_for(uint16_t raw) {
  if (raw <=  100)  return -1; // none (all off)
  if (raw <= 1400)  return 0;  // node1
  if (raw <= 2800)  return 1;  // node2
  return 2;
}

static bool send_led_cmd(unsigned id, byte cmd) {
  byte payload[1] = { cmd };
  return CAN.sendMsgBuf(id, 0, 1, payload) == CAN_OK;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  analogReadResolution(12);

  Wire.begin(); 
  Wire.setClock(400000);               // faster I2C for snappier refresh
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // If this prints, check wiring and 0x3C vs 0x3D address
    Serial.println("SSD1306 init failed");
  } else {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Hello OLED"));
    display.display();
    delay(500);
  }

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init FAIL"); while (1) {}
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("TX ready");
}

void loop() {
  static int lastTarget = -1;
  static uint32_t lastSampleMs = 0;
  const uint32_t samplePeriodMs = 50;

  uint32_t now = millis();
  if (now - lastSampleMs < samplePeriodMs) return;
  lastSampleMs = now;

  uint16_t raw = avg_adc(8, POT_PIN);   // 0..4095
  drawOLED(raw);

  int target = band_for(raw);
  if (target != lastTarget) {
    const unsigned ids[3] = { ID_NODE1, ID_NODE2, ID_NODE3 };  // <-- 3 entries

    if (lastTarget >= 0 && lastTarget <= 2) {
      send_led_cmd(ids[lastTarget], LED_OFF);
    }
    if (target >= 0 && target <= 2) {
      send_led_cmd(ids[target], LED_ON);
    }

    digitalWrite(LED_PIN, (target == 0) ? HIGH : LOW);  // mirror local LED for node1

    lastTarget = target;
  }
}
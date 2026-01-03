#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define DRAW_H 20
#define DRAW_W 88
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static const char* ITEMS[] = {"Timer", "Battery", "Volume"};
static const int ITEM_COUNT = sizeof(ITEMS) / sizeof(ITEMS[0]);
constexpr uint16_t adc_menu_pin    = 33;
constexpr uint16_t menu_button_pin = 32;

typedef struct {
  uint8_t selected;   // 0..ITEM_COUNT-1
  uint16_t raw;       // optional debug
} PotMsg;

struct Debounce {
  uint32_t lastChange = 0;
  bool lastStable = HIGH;  // for INPUT_PULLUP: idle = HIGH
  bool lastRead   = HIGH;
};

Debounce db_menu;

// Display mode controlled by button events
enum class ScreenMode : uint8_t { Menu, Hello };

typedef struct {
  ScreenMode mode;
} BtnMsg;

// Queues
static QueueHandle_t potQueue;
static QueueHandle_t btnQueue;

// -------- Helpers --------

static uint8_t bucketFromAdc(uint16_t raw) {
  // ESP32 ADC typically 0..4095 with 12-bit resolution
  uint32_t idx = (uint32_t)raw * ITEM_COUNT / 4096;
  if (idx >= (uint32_t)ITEM_COUNT) idx = ITEM_COUNT - 1;
  return (uint8_t)idx;
}

// Returns true only on a stable press edge.
// With INPUT_PULLUP wiring, "pressed" == LOW.
static bool pressedEdge(uint8_t pin, Debounce &db, uint16_t debounce_ms = 15) {
  bool raw = digitalRead(pin);
  uint32_t now = millis();

  if (raw != db.lastRead) {
    db.lastRead = raw;
    db.lastChange = now;
  }

  if (now - db.lastChange > debounce_ms) {
    if (raw != db.lastStable) {
      db.lastStable = raw;
      return (db.lastStable == LOW); // press edge
    }
  }
  return false;
}

// -------- Tasks --------

void PotentiometerTask(void *pv) {
  PotMsg msg{};
  uint8_t lastSel = 255;

  for (;;) {
    uint16_t raw = analogRead(adc_menu_pin);
    uint8_t sel = bucketFromAdc(raw);

    if (sel != lastSel) {
      lastSel = sel;
      msg.selected = sel;
      msg.raw = raw;
      xQueueOverwrite(potQueue, &msg); // latest wins
    }

    vTaskDelay(pdMS_TO_TICKS(25)); // ~40Hz
  }
}

// NEW: button sampling task (fast)
void ButtonTask(void *pv) {
  BtnMsg out{};
  ScreenMode mode = ScreenMode::Menu;

  for (;;) {
    if (pressedEdge(menu_button_pin, db_menu, 15)) {
      // Toggle mode on each press
      mode = (mode == ScreenMode::Menu) ? ScreenMode::Hello : ScreenMode::Menu;
      out.mode = mode;

      // Latest wins is fine here too
      xQueueOverwrite(btnQueue, &out);

      Serial.println("pressed"); // debug
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // sample button ~200Hz
  }
}

void DisplayTask(void* pv) {
  PotMsg pot{};
  BtnMsg btn{};
  uint8_t selected = 0;
  ScreenMode mode = ScreenMode::Menu;

  for (;;) {
    // Drain latest pot state if available
    if (xQueueReceive(potQueue, &pot, 0) == pdTRUE) {
      selected = pot.selected;
    }

    // Drain latest button state if available
    if (xQueueReceive(btnQueue, &btn, 0) == pdTRUE) {
      mode = btn.mode;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    if (mode == ScreenMode::Hello) {
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println("Hello");
      display.setTextSize(1);
      display.setCursor(0, 24);
      display.println("Button toggled");
    } else {
      // Menu screen
      display.setTextSize(1);

      for (int i = 0; i < ITEM_COUNT; i++) {
        int y = DRAW_H * i;

        if (i == selected) {
          display.fillRect(DRAW_H, y, DRAW_W, 16, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
        } else {
          display.drawRect(DRAW_H, y, DRAW_W, 16, SSD1306_WHITE);
          display.setTextColor(SSD1306_WHITE);
        }

        display.setCursor(DRAW_H + 5, y + 4);
        display.print(ITEMS[i]);
      }
    }

    display.display();
    vTaskDelay(pdMS_TO_TICKS(50)); // UI refresh
  }
}

void setup() {
  Serial.begin(115200);

  // IMPORTANT: INPUT_PULLUP assumes your button connects pin -> GND when pressed
  pinMode(menu_button_pin, INPUT_PULLUP);

  analogSetPinAttenuation(adc_menu_pin, ADC_11db);
  analogReadResolution(12);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    while (1) { delay(10); }
  }

  // Queues (length 1 because we only care about latest state)
  potQueue = xQueueCreate(1, sizeof(PotMsg));
  btnQueue = xQueueCreate(1, sizeof(BtnMsg));

  xTaskCreate(PotentiometerTask, "PotTask", 2048, nullptr, 2, nullptr);
  xTaskCreate(ButtonTask,        "BtnTask", 2048, nullptr, 2, nullptr);
  xTaskCreate(DisplayTask,       "DispTask", 4096, nullptr, 1, nullptr);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define BOX_X 20
#define BOX_W 88
#define ITEM_H 20
#define BOX_H 16
#define OLED_RESET -1           // No reset pin on many I2C modules; share MCU reset
#define OLED_ADDR 0x3C
#define RTC_ADDR 0x68

constexpr uint16_t ADCMAX = 4095; 
constexpr float_t VREF = 3.3; 
constexpr float_t MIN_BAT = 2; 
constexpr float_t MAX_BAT = 3.2; 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DateTime now; 
RTC_DS3231 rtc;
static uint32_t sirenOffAt = 0; 

static const char* ITEMS[] = {"Timer", "Battery", "Volume"};
static const int ITEM_COUNT = sizeof(ITEMS)/sizeof(ITEMS[0]);

constexpr uint16_t BAT_PIN = 13;  // measures battery (not wired yet)
constexpr uint16_t time_nxt_btn = 25; // next time slot in sub menu
constexpr uint16_t PWM_pin = 26;      // sends signal to mosfet -> fire alarm 
constexpr uint16_t siren_btn_pin = 27;  // fires siren 
constexpr uint16_t menu_button_pin = 32; // clicks menu
constexpr uint16_t time_inc_btn = 33; // inc time in sub menu
constexpr uint16_t VOL_POT = 34;      // controls volume through PWM intensity (not in use yet)
constexpr uint16_t ADC_MENU_POT = 35; // controls readings in menu by adc value measured


enum class UiEventType : uint8_t{
  HighlightChanged, 
  SelectPressed, 
  IncPressed, 
  NextPressed,
  SirenPressed
};

enum class Screen : uint8_t{
  Menu, Timer, Battery, Volume
}; 

struct UiEvent{
  UiEventType type; 
  uint8_t index; 
};  

struct Debounce {
  uint32_t lastChange = 0;
  bool lastStable = true; 
  bool lastRead   = true;
};

// debounce for different buttons
Debounce db_menu; 
Debounce db_inc; 
Debounce db_nxt;
Debounce db_pwm; 

struct TimeHM { uint8_t h=0, m=0; }; // default set times

struct SchedulerState {
  TimeHM start, end;
  uint8_t field = 0;      // 0:SH 1:SM 2:EH 3:EM 4:SAVE (optional)
};
static SchedulerState sched; 
static QueueHandle_t uiEventQ;

static uint8_t bucketFromAdc(uint16_t raw) {
  // ESP32 ADC 0 - 4095
  // Convert to 0..ITEM_COUNT-1
  uint32_t idx = (uint32_t)raw * ITEM_COUNT / 4096;
  if (idx >= (uint32_t)ITEM_COUNT){ 
    idx = ITEM_COUNT - 1;
  }
  return (uint8_t)idx;
}

void PotentiometerTask(void *pv) {
  uint8_t lastSel = 255;

  for (;;) {
    uint16_t raw = analogRead(ADC_MENU_POT);
    //Serial.println(analogRead(adc_menu_pin));
    uint8_t sel = bucketFromAdc(raw);

    if (sel != lastSel) {
      lastSel = sel;
      UiEvent evnt; 
      evnt.type = UiEventType::HighlightChanged; 
      evnt.index = sel; 
      if(xQueueSend(uiEventQ, &evnt, 0)!=pdTRUE){
        UiEvent junk; 
        xQueueReceive(uiEventQ, &junk, 0); 
        xQueueSend(uiEventQ, &evnt, 0); 
      }
    }
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

static bool pressedEdge(uint8_t pin, Debounce &db, uint16_t debounce_ms = 15) {
  bool raw = digitalRead(pin);                
  uint32_t now = millis();
  if (raw != db.lastRead) {                   //  input changed
    db.lastRead = raw;
    db.lastChange = now;
  }
  if (now - db.lastChange > debounce_ms) {    
    if (raw != db.lastStable) {         // new stable state
      db.lastStable = raw;
      return (db.lastStable == LOW);   //true per press
    }
  }
  return false;
}

void ButtonTask(void *pv){
  
  for(;;){
    if(pressedEdge(menu_button_pin, db_menu, 15)){
      UiEvent ev1; 
      //Serial.print("menu button pressed"); 
      ev1.type = UiEventType::SelectPressed; 
      ev1.index = 0; 

      //Serial.println("pressed"); 
      if(xQueueSend(uiEventQ, &ev1, 0)!= pdTRUE){
        UiEvent junk; 
        xQueueReceive(uiEventQ, &junk, 0);
        xQueueSend(uiEventQ, &ev1, 0); 
      }
    }
    if(pressedEdge(time_inc_btn, db_inc, 15)){
      UiEvent ev2; 
      ev2.type = UiEventType::IncPressed; 
      if(xQueueSend(uiEventQ, &ev2, 0)!= pdTRUE){
        UiEvent junk; 
        xQueueReceive(uiEventQ, &junk, 0); 
        xQueueSend(uiEventQ, &ev2, 0); 
      }
    }
    if(pressedEdge(time_nxt_btn, db_nxt, 15)){
      UiEvent ev3; 
      ev3.type = UiEventType::NextPressed; 
      if(xQueueSend(uiEventQ, &ev3, 0)!= pdTRUE){
        UiEvent junk; 
        xQueueReceive(uiEventQ, &junk, 0); 
        xQueueSend(uiEventQ, &ev3, 0); 
      }
    }
    if(pressedEdge(siren_btn_pin, db_pwm, 15)){
      UiEvent ev4; 
      //Serial.print(digitalRead(siren_btn_pin));
      ev4.type = UiEventType::SirenPressed;
      if(xQueueSend(uiEventQ, &ev4, 0)!= pdTRUE){
        UiEvent junk;   
        xQueueReceive(uiEventQ, &junk, 0); 
        xQueueSend(uiEventQ, &ev4, 0); 
        //Serial.print(digitalRead(siren_btn_pin));
      } 
    }
   vTaskDelay(pdMS_TO_TICKS(5));  
  }
}

static void drawMenu(uint8_t selected) {
  display.clearDisplay();
  display.setTextSize(1);

  for (int i = 0; i < ITEM_COUNT; i++) {
    int y = ITEM_H * i;

    if (i == selected) {
      display.fillRect(BOX_X, y, BOX_W, BOX_H, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.drawRect(BOX_X, y, BOX_W, BOX_H, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(BOX_X + 5, y + 4);
    display.print(ITEMS[i]);
  }

  display.display();
}

static void print2(Adafruit_SSD1306 &d, uint8_t v) {
  if (v < 10) d.print('0');
  d.print(v);
}

static void drawFieldBox(int16_t x, int16_t y, uint8_t val, bool selected) { // x, y, 
  const int16_t w = 26, h = 16;
  if (selected) {
    display.fillRect(x, y, w, h, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.drawRect(x, y, w, h, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(x + 5, y + 4);
  print2(display, val);
  // restore white, goe sback to black as default when rendering new page/submenu
  display.setTextColor(SSD1306_WHITE);
}

static void drawTimeScheduler(const DateTime& now, const SchedulerState& s){ // create schedule time to have alarm fire, set time and display current time. 
  display.clearDisplay(); 
  display.setTextColor(SSD1306_WHITE); 
  display.setTextSize(1);
  display.setCursor(14, 0);
  display.print(F("Time Scheduler"));
  display.setCursor(32,8); 
  if (now.hour()  < 10) display.print('0'); display.print(now.hour());  display.print(":");
  if (now.minute()< 10) display.print('0'); display.print(now.minute());display.print(":");
  if (now.second()< 10){ display.print('0'); display.println(now.second());}
  else display.println(now.second());  

  int16_t xHH = 32, yStart = 24, yEnd = 48;
  // starting HH:MM boxes
  drawFieldBox(xHH, yStart, s.start.h, (s.field == 0));
  drawFieldBox(xHH + 32, yStart, s.start.m, (s.field == 1));

  // End HH:MM boxes
  drawFieldBox(xHH, yEnd, s.end.h, (s.field == 2));
  drawFieldBox(xHH + 32, yEnd, s.end.m,(s.field == 3));

  // : 
  display.setCursor(xHH + 26, yStart + 4); display.print(':');
  display.setCursor(xHH + 26, yEnd   + 4); display.print(':');

  display.display(); 
}

static void drawVolumeBar(){
  int raw = analogRead(VOL_POT);                    
  int w = map(raw, 0, ADCMAX, 0, SCREEN_WIDTH);     // remaps v to range of 0-4095 to the width of 0 to screen width
  w = constrain(w,0,SCREEN_WIDTH);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Volume: ");
  int volume_range = map(raw, 0, ADCMAX, 0, 100);
  display.print(volume_range);

  display.fillRect(0, 20, w, 14, SSD1306_WHITE);
  display.drawRect(0, 20, SCREEN_WIDTH, 14, SSD1306_BLACK); // change w to be affected by potentiometer. 
  display.display();
}

static void drawBatteryLife(){   // measure Rtc battery with adc 
  int voltage = analogRead(BAT_PIN); // cr2032 battery is considered dead between 2-2.7 volts, starts around 3-3.2, read voltage when power on only
  float actualVoltage = voltage * (VREF / 1023.0); // returns real voltage
  float mapped_battery = map(actualVoltage, MIN_BAT ,MAX_BAT, 0, 100); // bar from 2 to 3.2
  display.clearDisplay(); 
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Battery Life: "));
  display.print(mapped_battery); 
  
  display.fillRect(0, 20, mapped_battery, 14, SSD1306_WHITE);
  display.drawRect(0, 20, SCREEN_WIDTH, 14, SSD1306_BLACK);
  display.display(); 
}

static void handleSchedulerEvent(UiEventType type, SchedulerState &s) {
  if (type == UiEventType::NextPressed) {
    //only changes which field is selected
    s.field = (s.field + 1) % 4;   // 0..3
    return;
  }

  if (type == UiEventType::IncPressed) {
    // only increments the currently-selected field
    switch (s.field) {
      case 0: s.start.h = (s.start.h + 1) % 24; break;
      case 1: s.start.m = (s.start.m + 1) % 60; break;
      case 2: s.end.h   = (s.end.h   + 1) % 24; break;
      case 3: s.end.m   = (s.end.m   + 1) % 60; break;
    }
  }
}

static int calculateDuty(){
  int raw = analogRead(VOL_POT); 
  int w = map(raw, 0, ADCMAX, 0, 255); 
  return w; 
} 

void UiTask(void *pv) {
  Screen screen = Screen::Menu;
  uint8_t highlightIndex = 0;
  static SchedulerState sched; 
 
  // Initial display 
  drawMenu(highlightIndex);

  for (;;) {
    UiEvent ev;
    if (screen == Screen::Timer) {
    now = rtc.now();
    }
    // Block 50ms waiting for events
    if (xQueueReceive(uiEventQ, &ev, pdMS_TO_TICKS(50)) == pdTRUE) {
      switch (ev.type) {
        case UiEventType::HighlightChanged:
          // Only change highlight while on Menu
          if (screen == Screen::Menu) {
            highlightIndex = ev.index;
          }
          break;

        case UiEventType::SelectPressed:
          // On menu enter the selected page
          // On page  go back to Menu
          Serial.println("menu button pressed"); 
          if (screen == Screen::Menu) {
            if (highlightIndex == 0) screen = Screen::Timer;
            else if (highlightIndex == 1) screen = Screen::Battery;
            else if (highlightIndex == 2) screen = Screen::Volume;
          } else {
            screen = Screen::Menu;
          }
          break;

        case UiEventType::IncPressed:
        case UiEventType::NextPressed:
          if (screen == Screen::Timer) {
            handleSchedulerEvent(ev.type, sched);
          }
          break;

        case UiEventType::SirenPressed:{
          Serial.println("Alarm button pressed"); 
          int duty = calculateDuty();
          analogWrite(PWM_pin, duty); 
          vTaskDelay(pdMS_TO_TICKS(500)); 
          analogWrite(PWM_pin, 0); 
          break; 
        }
      }
    }
    // render based on current stat
    switch (screen) {
      case Screen::Menu:    drawMenu(highlightIndex); break;
      case Screen::Timer:   drawTimeScheduler(now, sched); break;
      case Screen::Battery: drawBatteryLife(); break;
      case Screen::Volume:  drawVolumeBar(); break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  if(!rtc.begin()){
    Serial.print("RTC init failed"); 
    vTaskDelay(pdMS_TO_TICKS(100)); 
  } 
  rtc.disable32K(); 
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.disableAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);
  if (rtc.lostPower()) { 
    Serial.println("RTC lost power, setting time from compile time..."); // Set from the time this sketch was compiled: 
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  } 

  pinMode(menu_button_pin, INPUT_PULLUP);
  pinMode(time_inc_btn, INPUT_PULLUP);
  pinMode(time_nxt_btn, INPUT_PULLUP); 
  pinMode(siren_btn_pin, INPUT_PULLUP); 
  pinMode(PWM_pin, OUTPUT); 
  digitalWrite(PWM_pin, LOW); 

  analogSetPinAttenuation(ADC_MENU_POT, ADC_11db);
  analogReadResolution(12);


  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    while (1) { vTaskDelay(10 / portTICK_PERIOD_MS); }
  }

  uiEventQ = xQueueCreate(8, sizeof(UiEvent)); 

  xTaskCreate(PotentiometerTask, "PotTask", 2048, nullptr, 2, nullptr);
  xTaskCreate(ButtonTask, "BtnTask", 2048, nullptr, 2, nullptr);
  xTaskCreate(UiTask, "DispTask", 4096, nullptr, 1, nullptr);
}

void loop() {
  // empty - let tasks run
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/*
remaining: 
Battery life reader 
Fire Siren
*/

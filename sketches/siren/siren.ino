#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>       
#include <Adafruit_SSD1306.h>  
#include <RTClib.h>

constexpr uint8_t POT_PIN = A0; // for OLED (setting schedule, battery % remaining) 26
constexpr uint8_t VOL_PIN = A1; // for volume control 27
constexpr uint8_t BAT_PIN = A2; // for reading adc for battery power
constexpr uint8_t BUTTON_PIN1 = 10; // button used for siren test
constexpr uint8_t BUTTON_PIN2 = 11; // button used for OLED selection with bool isButtonPressed()
constexpr uint8_t BUTTON_PIN3 = 12; // used to adjust timer
constexpr uint8_t BUTTON_PIN4 = 13; // NEXT button to adjust timer 
// pins 4 & 5 i^2c sda scl
constexpr uint8_t PWM_PIN = 7;
constexpr uint32_t DT  = 200;   // debounce 
constexpr uint32_t ST = 500; 
constexpr uint16_t ADCMAX = 1023; // 2^8 = 256, use uint16
constexpr float_t VREF = 3.3; 

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define DRAW_H 20
#define DRAW_W 88
#define OLED_RESET -1           // No reset pin on many I2C modules; share MCU reset
#define OLED_ADDR 0x3C
#define RTC_ADDR 0x68

// pages -> set timer, set volume
// pages controlled by 2 buttons and a potentiometer
static const char* ITEMS[] = {"Timer", "Battery", "Volume"};
static const int ITEM_COUNT = sizeof(ITEMS)/sizeof(ITEMS[0]);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
enum class Page: int{Menu, Scheduler, Battery, Volume, SetTime};
Page activePage = Page::Menu;      
uint8_t menuIndex = 0; 
uint8_t subMenuIndex = 0; 

struct Debounce {
  uint32_t lastChange = 0;
  bool lastStable = true;  // INPUT_PULLUP idle = HIGH
  bool lastRead   = true;  // last raw read
};

struct TimeHM { uint8_t h=0, m=0; }; // default set times
struct SchedulerState {
  TimeHM start, end;
  uint8_t field = 0;      // 0:SH 1:SM 2:EH 3:EM 4:SAVE (optional)
};

SchedulerState nxt; 
Debounce db_siren;   // button 1
Debounce db_menu;   // butto 2
Debounce db_inc;   // button 3
Debounce db_next; // button 4
RTC_DS3231 rtc;
static uint32_t sirenOffAt = 0;

DateTime now; 

void displaySetup(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 init failed"));
    while (1) {}
  } 
  display.clearDisplay();
  display.setTextSize(2);            // 2x scale
  display.setTextColor(SSD1306_WHITE); // monochrome
  display.setCursor(0, 0);
  display.println(F("Hello OLED"));
  display.setTextSize(1);
  display.println(F("Pi Pico W + I^2C"));
  display.display();
  delay(1200);
}

int adcToMenuIndex(int adc) {
  // 3 equal buckets (0..1023) -> 0,1,2 with hysteresis
  static uint8_t sel = 0;
  static int lastChange = 0;
  int div = map(adc, 0, 1023, 0, (int)ITEM_COUNT -1); // range of 0 to 2

  const int HYST = 30; // ADC counts (~3%), prevents from drastic/shaky changes in OELD
  if (div != sel) {
    if (abs(adc - lastChange) > HYST) { sel = div; lastChange = adc; }
  }
  return sel;
}

int adcToSubMenuIndex(int adc){
    // when setting time in the TImeSchedule page, make sub menu from 6 buckets, be able to press a button hovering over them to increment display by 1 at a time
    // add 1 more button for incrementing numbers, loop over 24. 
    // [HOUR]     [MINUTE]      [SECOND] 
    //            ___to___
    // [HOUR]     [MINUTE]      [SECOND] 
  static uint8_t sel = 0;
  static int lastChange = 0;
  int div = map(adc, 0, 1023, 0, 5); 
  const int HYST = 30; // ADC counts (~3%), prevents from drastic/shaky changes in OELD
  if (div != sel) {
    if (abs(adc - lastChange) > HYST) { sel = div; lastChange = adc; }
  }
  return sel; // returns 0 -5
}

bool pressedEdge(uint8_t pin, Debounce &db, uint16_t debounce_ms = 15) {
  bool raw = digitalRead(pin);                // HIGH idle pullup
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

void drawMenu(int selected){
    display.clearDisplay();
    display.setTextSize(1);
    //int w = map(raw, 0, ADCMAX, 0, SCREEN_WIDTH);     // remaps v to range of 0-1023 to the width of 0 to screen width
    //w = constrain(w,0,SCREEN_WIDTH);

    for(int i = 0; i < ITEM_COUNT ; i++){
        display.drawRect(DRAW_H, 20*i, DRAW_W, 16, SSD1306_WHITE);               // (20,0) -> (20,20) = (+0, +20)
        display.setCursor(25, 5 +(20*i));             // (x,y) -> (0, 5) -> (+0, +20)
        display.print(ITEMS[i]);
        if (i == selected) {
        display.print((i == selected) ? "> " : "  ");
        }
    }
   display.display(); 
}

void drawBatteryLife(){   // measure Rtc battery with adc 
  int voltage = analogRead(BAT_PIN); // cr2032 battery is considered dead between 2-2.7 volts, starts around 3-3.2, read voltage when power on only
  float actualVoltage = voltage * (VREF / 1023.0); // returns real voltage
  float v = map(actualVoltage, 0,ADCMAX, 0, 3.3);
   
  display.clearDisplay(); 
  display.setTextSize(1);
  display.setCursor(10, 10);
  display.print(F("Battery Life: "));
  display.print(v); 
  display.display(); 
}

void printDateTime(const DateTime& now) {
  static const char* DOW[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  Serial.print(DOW[now.dayOfTheWeek()]); Serial.print(" ");
  Serial.print(now.year());  Serial.print("-");
  if (now.month() < 10) Serial.print('0'); Serial.print(now.month()); Serial.print("-");
  if (now.day()   < 10) Serial.print('0'); Serial.print(now.day());   Serial.print(" ");
  if (now.hour()  < 10) Serial.print('0'); Serial.print(now.hour());  Serial.print(":");
  if (now.minute()< 10) Serial.print('0'); Serial.print(now.minute());Serial.print(":");
  if (now.second()< 10){ Serial.print('0'); Serial.println(now.second());}
  else Serial.println(now.second()); 
}

void handleSchedulerInput(uint8_t pinInc, Debounce& dbInc, uint8_t pinNext, Debounce& dbNext, SchedulerState& s){
  if (pressedEdge(pinNext, dbNext)) {
    s.field = (s.field + 1) % 4;   // 4 fields (SH,SM,EH,EM)
  }

  if (pressedEdge(pinInc, dbInc)) {
    switch (s.field) {
      case 0: s.start.h = (s.start.h + 1) % 24; break;
      case 1: s.start.m = (s.start.m + 1) % 60; break;
      case 2: s.end.h   = (s.end.h   + 1) % 24; break;
      case 3: s.end.m   = (s.end.m   + 1) % 60; break;
    }
  Serial.print("Start time:");  if (s.start.h < 10) Serial.print('0'); Serial.print(s.start.h);
  Serial.print(":");   if (s.start.m < 10) Serial.print('0'); Serial.print(s.start.m);
  Serial.print("\n"); 
  Serial.print("End Time:");if (s.end.h   < 10) Serial.print('0'); Serial.print(s.end.h);
  Serial.print(":");   if (s.end.m   < 10) Serial.print('0'); Serial.print(s.end.m);
  Serial.print("\n");
  }
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
  // restore white for other text
  display.setTextColor(SSD1306_WHITE);
}

void drawTimeScheduler(const DateTime& now, const SchedulerState& s){ // create schedule time to have alarm fire, set time and display current time. 
  display.clearDisplay(); 
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

void drawVolumeBar(){
  int raw = analogRead(VOL_PIN);                    
  int w = map(raw, 0, ADCMAX, 0, SCREEN_WIDTH);     // remaps v to range of 0-1023 to the width of 0 to screen width
  w = constrain(w,0,SCREEN_WIDTH);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Volume: ")); // use F to save RAM on uno
  int volume_range = map(raw, 0, ADCMAX, 0, 100);
  display.print(volume_range);

  display.fillRect(0, 20, w, 14, SSD1306_WHITE);
  display.drawRect(0, 20, SCREEN_WIDTH, 14, SSD1306_BLACK); // change w to be affected by potentiometer. 
  display.display();
}

void sel_page(int raw, const DateTime& now){
  switch (activePage) {
    case Page::Menu:{
      menuIndex = adcToMenuIndex(raw);
     drawMenu(menuIndex); break;
     break;
    }
    case Page::Scheduler:{
      subMenuIndex = adcToSubMenuIndex(raw);
      drawTimeScheduler(now, nxt);  
      break;
    }
    case Page::Volume:     drawVolumeBar(); break; 
    case Page::Battery:   drawBatteryLife(); break;
  }
}

void scheduleAlarm10s() {       // alarm goes off every 10 seconds, test - will change 
  DateTime next = rtc.now() + TimeSpan(0,0,0,10);
  rtc.clearAlarm(1);                        // clear flag before setting a new one
  rtc.setAlarm1(next, DS3231_A1_Date);     //set at timestamp 
}

void alarm_loop(uint8_t duty, uint32_t duration){ // alarm fnx called in loop() that turns on s
  if (rtc.alarmFired(1)) {
    // Ssiren on 
    analogWrite(PWM_PIN, duty);
    // Re-arm for 10s later
    scheduleAlarm10s();
    sirenOffAt = millis() + duration; 
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  rtc.begin(); 
  rtc.disable32K(); // not using 32k pin 
  //pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);           // i2c components setup in wire.begin(), only in setup() for sqw pin for itnerrupts. 
  //attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.disableAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);
  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP); //idle high
  pinMode(BUTTON_PIN3, INPUT_PULLUP); 
  pinMode(BUTTON_PIN4, INPUT_PULLUP); 
  pinMode(PWM_PIN, OUTPUT); 
  digitalWrite(PWM_PIN, LOW); 
  analogReadResolution(10);
  
  delay(100);
  displaySetup(); 
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //DateTime now = rtc.now();
  DateTime next = now + TimeSpan(0,0,0,10);
  rtc.setAlarm1(next, DS3231_A1_Date);
}

void loop() {
  static uint8_t  pressCount = 0;
   
  DateTime now = rtc.now(); 
  
  bool sirenPress = pressedEdge(BUTTON_PIN1, db_siren);
  bool menuPress  = pressedEdge(BUTTON_PIN2, db_menu);
  //alarm_loop(duty, ST);  // annoying when left on
  // printDateTime(now); 
  if (menuPress && activePage != Page::Menu) {
    activePage = Page::Menu;
    // render and exit 
    int rawReturn = analogRead(POT_PIN);
    sel_page(rawReturn, now);
    delay(10);
    return;
  }
  if (activePage == Page::Scheduler) { // call handle schedule if page is sub menu scheduler 
      handleSchedulerInput(BUTTON_PIN3, db_inc, BUTTON_PIN4, db_next, nxt);
  } 
    
  else { // if not submnu schedule, do normal OLED routine 
      if (menuPress) {
        if (activePage == Page::Menu) {
          switch (menuIndex) {
            case 0: activePage = Page::Scheduler; break;
            case 1: activePage = Page::Battery;   break;
            case 2: activePage = Page::Volume;    break;
            default: activePage = Page::Menu;     break;
          }
        } else {
          activePage = Page::Menu;
        }
      }

      // 4-press siren test
      if (sirenPress) {
        if (++pressCount >= 4) {
          int duty = map(analogRead(VOL_PIN), 0, ADCMAX, 0, 255);
          duty = constrain(duty, 0, 255);
          analogWrite(PWM_PIN, duty);
          sirenOffAt = millis() + ST;
          pressCount = 0;
        }
      }
  }

  if (sirenOffAt && (int32_t)(millis() - sirenOffAt) >= 0) {
    analogWrite(PWM_PIN, 0);
    sirenOffAt = 0;
  }
  int raw = analogRead(POT_PIN); // placed at bottom to render new state
  sel_page(raw, now);
  delay(10); // small delay
}

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>       
#include <Adafruit_SSD1306.h>  
#include <RTClib.h>

constexpr uint8_t POT_PIN = A0; // for OLED (setting schedule, battery % remaining) 26
constexpr uint8_t VOL_PIN = A1; // for volume control 27
constexpr uint8_t BUTTON_PIN1 = 2; // button used for siren test
constexpr uint8_t BUTTON_PIN2 = 3; // button used for OLED selection with bool isButtonPressed()
// pins 4 & 5 i^2c sda scl
constexpr uint8_t PWM_PIN = 7;
constexpr uint32_t DT  = 200;   // debounce 
constexpr uint32_t ST = 500; 
constexpr uint16_t ADCMAX = 1023; // 2^8 = 256, use uint16

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
enum class Page: int{Menu, Scheduler, Battery, Volume};
Page activePage = Page::Menu;      
uint8_t menuIndex = 0; 

struct Debounce {
  uint32_t lastChange = 0;
  bool lastStable = true;  // INPUT_PULLUP idle = HIGH
  bool lastRead   = true;  // last raw read
};

Debounce db_siren;   // button 1
Debounce db_menu;   // butto 2
RTC_DS3231 rtc;
static uint32_t sirenOffAt = 0;


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
  display.clearDisplay(); 
  display.setTextSize(1);
  display.setCursor(10, 10);
  display.print(F("Battery Life"));
  display.display(); 
}

void drawTimeScheduler(){ // create scheduled time to have alarm go off using rtc
  display.clearDisplay(); 
  display.setTextSize(1);
  display.setCursor(10, 10);
  display.print(F("Time Scheduler"));
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

void sel_page(int raw){
  menuIndex = adcToMenuIndex(raw); // returns 0,1,2
  switch (activePage) {
    case Page::Menu:      drawMenu(menuIndex); break;
    case Page::Scheduler: drawTimeScheduler();  break;
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
  pinMode(PWM_PIN, OUTPUT); 
  digitalWrite(PWM_PIN, LOW); 
  analogReadResolution(10);
  
  delay(100);
  displaySetup(); 

  DateTime now = rtc.now();
  DateTime next = now + TimeSpan(0,0,0,10);
  rtc.setAlarm1(next, DS3231_A1_Date);
}

void loop() {
  static uint8_t  pressCount = 0;
 

  int raw = analogRead(POT_PIN);
  sel_page(raw);  

  int raw2 = analogRead(VOL_PIN);
  int duty = map(raw2, 0, ADCMAX, 0, 255); 
  duty = constrain(duty, 0, 255);
  
  bool menuPress  = pressedEdge(BUTTON_PIN2, db_menu);
  bool sirenPress = pressedEdge(BUTTON_PIN1, db_siren);

  alarm_loop(duty, ST); 

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
  // siren 4 press
  if (sirenPress) {
    if (++pressCount >= 4) {
      analogWrite(PWM_PIN, duty);          // fixed duty for now
      sirenOffAt = millis() + ST;
      pressCount = 0;
    }
  }
  if (sirenOffAt && (int32_t)(millis() - sirenOffAt) >= 0) {
    analogWrite(PWM_PIN, 0);
    sirenOffAt = 0;
  }
}

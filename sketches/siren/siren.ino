#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>       
#include <Adafruit_SSD1306.h>  

constexpr uint8_t POT_PIN = A0; 
constexpr uint8_t BUTTON_PIN1 = 2;
constexpr uint8_t BUTTON_PIN2 = 3;
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

// pages -> set timer, set volume
// pages controlled by 2 buttons and a potentiometer
static const char* ITEMS[] = {"Menu", "Time Scheduler", "Volume"};
static const int ITEM_COUNT = sizeof(ITEMS)/sizeof(ITEMS[0]);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 
enum class Page: int{Menu, Scheduler, Volume};
Page activePage = Page::Menu;      
uint8_t menuIndex = 0; 

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
  display.println(F("Arduino UNO + I^2C"));
  display.display();
  delay(1200);
}

int adcToMenuIndex(int adc) {
  // 3 equal buckets (0..1023) -> 0,1,2 with hysteresis
  static uint8_t sel = 0;
  static int lastChange = 0;
  int div = map(adc, 0, 1023, 0, (int)ITEM_COUNT -1); // range of 0 to 3

  const int HYST = 30; // ADC counts (~3%), prevents from drastic/shaky changes in OELD
  if (div != sel) {
    if (abs(adc - lastChange) > HYST) { sel = div; lastChange = adc; }
  }
  return sel;
}

void drawMenu(int selected){
    display.clearDisplay();
    display.setTextSize(1);
    //int w = map(raw, 0, ADCMAX, 0, SCREEN_WIDTH);     // remaps v to range of 0-1023 to the width of 0 to screen width
    //w = constrain(w,0,SCREEN_WIDTH);

    for(int i = 0; i < ITEM_COUNT; i++){
        display.drawRect(DRAW_H, 20*i, DRAW_W, 16, SSD1306_WHITE);               // (20,0) -> (20,20) = (+0, +20)
        display.setCursor(45, 5 +(20*i));             // (x,y) -> (0, 5) -> (+0, +20)
        display.print(ITEMS[i]);

        if (i == selected) {
        display.fillRect(DRAW_H, 20*i, DRAW_W, 16, SSD1306_BLACK);
  }
    }

   display.display(); 
}

void drawTimeScheduler(){
  display.clearDisplay(); 
  display.setTextSize(1);
  display.setCursor(10, 10);
  display.print(F("Time Scheduler"));
  display.display(); 
}

void drawVolumeBarScreen(){
  int raw = analogRead(POT_PIN); 
  int w = map(raw, 0, ADCMAX, 0, SCREEN_WIDTH);     // remaps v to range of 0-1023 to the width of 0 to screen width
  w = constrain(w,0,SCREEN_WIDTH);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("A0: ")); // use F to save RAM on uno
  display.print(raw);

  display.fillRect(0, 20, w, 14, SSD1306_WHITE);
  display.drawRect(0, 20, SCREEN_WIDTH, 14, SSD1306_BLACK); // change w to be affected by potentiometer. 
  display.display();
}

bool readButtonPressed() {
  static uint32_t lastChange = 0;
  static bool lastStable = false;
  static bool lastRead = true;
  bool raw = digitalRead(BUTTON_PIN2); // true if pressed
  if (raw != lastRead) { lastRead = raw; lastChange = millis(); } ///updates last read time
  if (millis() - lastChange > 15) {  // debounce 15ms
    if (raw != lastStable) { lastStable = raw; return (lastStable == LOW); } // returns true
  }
  return false;
}

void sel_page(int raw){
    switch(activePage){

    case Page::Menu: {
      // pot only moves HIGHLIGHT, never changes the page:
      menuIndex = adcToMenuIndex(raw); // returns int 
      drawMenu(menuIndex);  // menu index is controlled by potentiometer
      if (readButtonPressed()) {
        activePage = (menuIndex == 0) ? Page::Scheduler
                    :  Page::Volume;                       
      }
    } break;
        
    case Page:: Scheduler:{
        drawTimeScheduler();
        if (readButtonPressed()) activePage = Page::Menu; 
    } break;

    case Page::Volume:{
        drawVolumeBarScreen(); // draws bars
        if(readButtonPressed()){
         // analogWrite(PWM_PIN, menuIndex);
          activePage = Page::Menu; 
        }
    } break;
    }
  //delay(100);
}

void setup() {
  Serial.begin(115200);
  digitalWrite(PWM_PIN, LOW); 
  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
  pinMode(PWM_PIN, OUTPUT); 
  analogReadResolution(10);
  Wire.begin();
  delay(100);
  displaySetup(); 
}

void loop() {
  static uint8_t  pressCount = 0;     // 0..3
  static bool     lastBtn    = HIGH;  // HIGH = not pressed
  static uint32_t lastEdgeMs = 0;
  static uint32_t sirenOffAt = 0;

  int raw = analogRead(POT_PIN); 
  sel_page(raw); 
  uint32_t now = millis();
  bool btn = digitalRead(BUTTON_PIN1);
  bool rising = (lastBtn == HIGH && btn == LOW);

  if (rising && (now - lastEdgeMs) > DT) {
    if (pressCount < 3) {
      ++pressCount;
    } 
    else { // 4th press: fire siren & reset
      analogWrite(PWM_PIN, 50);
      sirenOffAt = now + ST;

      pressCount = 0;
    }
    lastEdgeMs = now;
  }
  lastBtn = btn;
  if (sirenOffAt && (int32_t)(now - sirenOffAt) >= 0) {
    analogWrite(PWM_PIN, 0);
    sirenOffAt = 0;
  }
}

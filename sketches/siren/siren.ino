#include <Arduino.h>

constexpr uint8_t LED_PINS[3] = {7, 8, 9};
constexpr uint8_t BUTTON_PIN = 2;
constexpr uint8_t PWM_PIN = 3; 
constexpr uint32_t DT  = 200;   // debounce 
constexpr uint32_t ST = 500; 

void sirenOn()  {digitalWrite(PWM_PIN, HIGH);}
void sirenOff() {digitalWrite(PWM_PIN, LOW);}


void setup() {
  digitalWrite(PWM_PIN, LOW); 
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PWM_PIN, OUTPUT); 
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
}

void loop() {
  static uint8_t  pressCount = 0;     // 0..3
  static bool     lastBtn    = HIGH;  // HIGH = not pressed
  static uint32_t lastEdgeMs = 0;
  static uint32_t sirenOffAt = 0;


  uint32_t now = millis();
  bool btn = digitalRead(BUTTON_PIN);
  bool rising = (lastBtn == HIGH && btn == LOW);


  if (rising && (now - lastEdgeMs) > DT) {
    if (pressCount < 3) {
      digitalWrite(LED_PINS[pressCount], HIGH);  // 1st→3rd LED
      ++pressCount;
    } 
    else { // 4th press: fire siren & reset
      analogWrite(PWM_PIN, 50);
      sirenOffAt = now + ST;

      for (uint8_t i = 0; i < 3; ++i) digitalWrite(LED_PINS[i], LOW);
      pressCount = 0;
    }
    lastEdgeMs = now;
  }
  lastBtn = btn;
  if (sirenOffAt && (int32_t)(now - sirenOffAt) >= 0) {
    digitalWrite(PWM_PIN, LOW);
    sirenOffAt = 0;
  }
}
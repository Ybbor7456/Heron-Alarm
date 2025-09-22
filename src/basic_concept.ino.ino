#include <Arduino.h>
constexp int pin_dor = 2;
constexp int baud; 

const int sensor[3] = 3; 


void rs485_send(const uint8_t* p, size_t n) {
  gpio_put(pin_dor, 1);                      // drive
  uart_write_blocking(uart0, p, n);          // takes uart instance, pointer to uart data, number of bytes
  while (uart_is_writable(uart0)) {}         
  delayMicroseconds(200);                     
  gpio_put(pin_dor, 0);                      // release
}


void setup() {
  Serial1.setFIFOSize(256);
  Serial1.begin(BAUD);
  pinMode(PIN_DE_RE, OUTPUT);
  digitalWrite(PIN_DE_RE, LOW);

  for (uint8_t p : SENS) pinMode(p, INPUT_PULLDOWN);
  gpio_init(PIN_DE_RE); gpio_set_dir(PIN_DE_RE, true); gpio_put(PIN_DE_RE, 0);
}


void loop(){
  int value = digitalRead(inputPin);
  if (value == HIGH) {
    digitalWrite(buzzerPin, HIGH);  
    delay(1000);                    
    digitalWrite(buzzerPin, LOW);   
}}

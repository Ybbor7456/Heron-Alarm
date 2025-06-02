const int buzzerPin = 12;
const int inputPin  = 2;

void setup(){
  pinMode(buzzerPin, OUTPUT);
  pinMode(inputPin, INPUT);
}

void loop(){
  int value = digitalRead(inputPin);
  if (value == HIGH) {
    digitalWrite(buzzerPin, HIGH);  
    delay(1000);                    
    digitalWrite(buzzerPin, LOW);   
}}
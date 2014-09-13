#include <arduino/Arduino.h>

void setup() {
  pinMode(13,OUTPUT);
}

void loop() {
  const unsigned int kDuration = 500;
  
  digitalWrite(13,HIGH);
  delay(kDuration);
  digitalWrite(13,LOW);
  delay(kDuration);
}

int main() {
  init();
  setup();
  
  while(1)
    loop();
}

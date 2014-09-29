#include <arduino/Arduino.h>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(9600);
  pinMode(13,OUTPUT);  
  
  while (true) {
    
    digitalWrite(13,HIGH);
    if(Serial.available()){
      digitalWrite(13,LOW);
      int x = Serial.read();    
    
      if(x==1) ;
    }
    
    delay(500);
  }
  
  Serial.end();
  return 0;
}

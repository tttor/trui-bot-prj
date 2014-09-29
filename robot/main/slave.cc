#include <arduino/Arduino.h>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(9600);
  pinMode(13,OUTPUT);  
  
  digitalWrite(13,HIGH);

  while (true) {
    
    if(Serial.available()>0){
      
      int x = Serial.read();    
      digitalWrite(13,LOW);

      if(x==1) digitalWrite(13,LOW);
      else if(x==2) digitalWrite(13,HIGH);
    }
    
    delay(500);
  }
  
  Serial.end();
  return 0;
}

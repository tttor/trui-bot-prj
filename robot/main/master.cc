#include <arduino/Arduino.h>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial2.begin(9600);  
 
  while (true) {

  //  Serial.print("Hello");
    //Serial2.print(1, DEC);   
    
    //delay(500);

    Serial2.print(2, DEC);   
    
  }
  
  Serial2.end();
  return 0;
}

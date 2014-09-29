#include <arduino/Arduino.h>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial1.begin(9600);  
  
  while (true) {

  //  Serial.print("Hello");
    Serial1.print("c");   
    
    delay(500);

    Serial1.print("a");   
    
    delay(500);
  }
  
  Serial1.end();
  return 0;
}

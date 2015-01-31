// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <arduino/Arduino.h>
#include <Wire.h>

int val=0;
uint8_t buffer[10];
int received_data;

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  int i = 0;
  while(0 < Wire.available()) // loop through all
  {
    buffer[i] = Wire.read(); // receive byte as a character
    i++;
  }

  int checksum= (buffer[8] << 8);
  checksum |= buffer[7];
  if(checksum == (buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[9])) Serial.println("OK");
  else Serial.println("ERROR");
}

int main() {
  init();
  pinMode(13,OUTPUT);
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event

  Serial.begin(9600);           // start serial for output

  while(1){
  if(buffer[2] == 0x0F)
    //SERVE!!!!!
  delay(100);
  }
}

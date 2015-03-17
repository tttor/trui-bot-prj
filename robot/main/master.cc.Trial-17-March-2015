#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/custom.h>

#define debugPin 13
#define hitPin 20
#define risePin 21
#define miscPneu1 22
#define miscPneu2 23
#define reset1 41
#define reset2 42
#define reset3 43

#define initialSpeed 0

void toggleReset(){

// digitalWrite(reset1,HIGH);
// digitalWrite(reset2,HIGH);
// digitalWrite(reset3,HIGH);
// delay(500);
digitalWrite(reset1,LOW);
digitalWrite(reset2,LOW);
digitalWrite(reset3,LOW);
delay(500);
digitalWrite(reset1,HIGH);
digitalWrite(reset2,HIGH);
digitalWrite(reset3,HIGH);
}

int main() {

    //digitalWrite(hitPin,LOW);

  //Timer0  
  //Setting   Divisor     Frequency
  //0x01        1           62500
  //0x02        8           7812.5
  //0x03        64          976.5625   <--DEFAULT
  //0x04        256         244.140625
  //0x05        1024        61.03515625

  //TCCR0B = TCCR0B & 0b11111000 | <setting>;
  const uint8_t header= 0xCE;
  const uint8_t footer= 0xEE;

  const uint8_t hit_mode= 0x0A;
  const uint8_t position_mode= 0x0B;
  const uint8_t hit_flag = 0x0F;
  const uint8_t rise_flag =0xF0;

  const uint8_t channel= MAVLINK_COMM_0;
  const uint8_t desired_msgid= MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;    

  uint8_t controllerBuffer[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0}; 

  init();
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);    
  Serial3.begin(9600);
  pinMode(hitPin,OUTPUT);
  pinMode(risePin,OUTPUT);
  pinMode(miscPneu1,OUTPUT);
  pinMode(miscPneu2,OUTPUT);

  pinMode(reset1,OUTPUT);
  pinMode(reset2,OUTPUT);
  pinMode(reset3,OUTPUT);
  digitalWrite(hitPin,LOW);
  digitalWrite(risePin,LOW);
  digitalWrite(miscPneu1,LOW);
  digitalWrite(miscPneu2,LOW);

  


  delay(1000);
  toggleReset();
  delay(500);
  
  while (true) {
    
    //1. wait for cmd_speed from serial from controller
    //while (true) {
      if (Serial.available() >= 14) {
        if(Serial.read() == 0xCE){ //Header
            controllerBuffer[0] = Serial.read(); //Feedback Flag for wheel 1
            controllerBuffer[1] = Serial.read(); //LSB of Wheel Speed 1
            controllerBuffer[2] = Serial.read(); //MSB of Wheel Speed 1
            controllerBuffer[3] = Serial.read(); //Feedback Flag for wheel 2
            controllerBuffer[4] = Serial.read(); //LSB of Wheel Speed 2
            controllerBuffer[5] = Serial.read(); //MSB of Wheel Speed 2
            controllerBuffer[6] = Serial.read(); //Feedback Flag for wheel 3
            controllerBuffer[7] = Serial.read(); //LSB of Wheel Speed 3
            controllerBuffer[8] = Serial.read(); //MSB of Wheel Speed 3
            controllerBuffer[9] = Serial.read(); //Hit Flag
            controllerBuffer[10] = Serial.read(); //LSB of Rotater
            controllerBuffer[11] = Serial.read(); //MSB of Rotater
            controllerBuffer[12] = Serial.read(); //Footer
            }
      }
    

    //2. break down the speed into the speed for 3 motors
    
    //if(controllerBuffer[9] == hit_flag){digitalWrite(hitPin,HIGH);}
    if((controllerBuffer[9] & 0x0F) == hit_flag){digitalWrite(hitPin,HIGH);} else digitalWrite(hitPin,LOW);
    if((controllerBuffer[9] & 0xF0) == rise_flag){digitalWrite(risePin,HIGH);} else digitalWrite(risePin,LOW);
    //3. dispatch and send the 3 speed values
    uint8_t buffer1[7];
    uint8_t buffer2[7];
    uint8_t buffer3[7];

    buffer1[0]= header;
    buffer1[6]= footer;
    buffer1[2]= controllerBuffer[2];
    buffer1[1]= controllerBuffer[1];
    buffer1[3]= controllerBuffer[0];
    uint16_t checksum1= buffer1[0] + buffer1[1] + buffer1[2] + buffer1[6];
    buffer1[4]= checksum1 & 0x00FF;
    buffer1[5]= (checksum1 & 0xFF00) >> 8;


    analogWrite(13,controllerBuffer[1]);

    buffer2[0]= header;
    buffer2[6]= footer;
    buffer2[2]= controllerBuffer[5];
    buffer2[1]= controllerBuffer[4];
    buffer2[3]= controllerBuffer[3];
    uint16_t checksum2= buffer2[0] + buffer2[1] + buffer2[2] + buffer2[6];
    buffer2[4]= checksum2 & 0x00FF;
    buffer2[5]= (checksum2 & 0xFF00) >> 8;


    buffer3[0]= header;
    buffer3[6]= footer;
    buffer3[2]= controllerBuffer[8];
    buffer3[1]= controllerBuffer[7];
    buffer3[3]= controllerBuffer[6];
    uint16_t checksum3= buffer3[0] + buffer3[1] + buffer3[2] + buffer3[6];
    buffer3[4]= checksum3 & 0x00FF;
    buffer3[5]= (checksum3 & 0xFF00) >> 8;

    //For normal operation
    Serial1.write(buffer1, 7);
    Serial2.write(buffer2, 7);
    Serial3.write(buffer3, 7);


    
    //For Testing
    // Serial.print(controllerBuffer[0]);
    // Serial.print("-");
    // Serial.print(controllerBuffer[1]);
    // Serial.print("-");
    // Serial.print(controllerBuffer[2]);
    // Serial.print("-");
    // Serial.print(controllerBuffer[3]);
    // Serial.print("-");
    // Serial.print(controllerBuffer[4]);
    // Serial.print("-");
    // Serial.print(controllerBuffer[5]);
    // Serial.print("-");
    // Serial.print(controllerBuffer[6]);
    // Serial.print("-");
    // Serial.print(controllerBuffer[7]);
    // Serial.print("-");
    // Serial.print(controllerBuffer[8]);
    // Serial.print("-");
    // Serial.print(controllerBuffer[9]);
    // Serial.print("-");
    // Serial.print(controllerBuffer[10]);
    // Serial.print("-");
    // Serial.print(controllerBuffer[11]);
    // Serial.print("-");
    // Serial.println(controllerBuffer[12]);
    // delay(10);

    // Serial.print(buffer1[0]);
    // Serial.print("-");
    // Serial.print(buffer1[1]);
    // Serial.print("-");
    // Serial.print(buffer1[2]);
    // Serial.print("-");
    // Serial.print(buffer1[3]);
    // Serial.print("-");
    // Serial.print(buffer1[4]);
    // Serial.print("-");
    // Serial.print(buffer1[5]);
    // Serial.print("-");
    // Serial.print(buffer1[6]);
    // Serial.print("-");
    // Serial.println(buffer1[7]);

  
  }
  return 0;
}


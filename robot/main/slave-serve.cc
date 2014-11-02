//Slave-Serve
#include <arduino/Arduino.h>
#include <DynamixelSerial/DynamixelSerial.h>
#include <arduino/Wire.h>


#define ARM_ID 15
#define GRIP_ID 14
#define DYNA_CON 2
#define PNEU_PIN 3

//Constants
#define RETRACT 0
#define EXTEND 1
#define SER_DELAY 210 //ms

int val=0;
uint8_t buffer[11];
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
  if(checksum == (buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[9])) //Serial.println("OK");
  //else Serial.println("ERROR");
}

//Positioning arm to service
void armDeploy(){
  int angle = 150-120;
  int angle_tr = (double)angle * 1023 / 300;
  Dynamixel.move(ARM_ID,angle_tr);
}

//Positioning arm to initialize hitter
void armRetract(){
  int angle = 150+120;
  int angle_tr = (double)angle * 1023 / 300;
  Dynamixel.move(ARM_ID,angle_tr);
}

//Dropping shuttlecock
void gripperOpen(){
  int angle = 150 + 60;
  int angle_tr = (double)angle * 1023 / 300;
    //Dynamixel.move(GRIP_ID,angle_tr);
  Dynamixel.move(GRIP_ID,0);
}

//Close the gripper
void gripperClose(){
  int angle = 150;
  int angle_tr = (double)angle * 1023 / 300;
    //Dynamixel.move(GRIP_ID,angle_tr);
  Dynamixel.move(GRIP_ID,512);

}

//Position the racket to bottom (ready for shooting)
void racketBottom(){
  digitalWrite(PNEU_PIN,RETRACT);
}

//Shot the racket up!
void racketUp(){
  digitalWrite(PNEU_PIN,EXTEND);
}

//Service routine
void serviceShot(){
  //armDeploy();
  armRetract();
  delay(2000);

  gripperOpen();
  delay(SER_DELAY);
  
  racketUp();
  delay(1000);
  
  racketBottom();
  gripperClose();
  armDeploy();
  //armRetract();
}

//initialization needed
void serviceInit(){
  Dynamixel.begin(1000000,2);
  Dynamixel.setEndless(ARM_ID,OFF);
  Dynamixel.setEndless(GRIP_ID,OFF);
  Dynamixel.setMaxTorque(ARM_ID,512);
  Dynamixel.setMaxTorque(GRIP_ID,512);
  Dynamixel.move(ARM_ID,240*1023/300);
  Dynamixel.move(GRIP_ID,150*1023/300);
  delay(1000);
  pinMode(PNEU_PIN,OUTPUT);
  digitalWrite(PNEU_PIN,HIGH);
  
}

int main(){
  init();
  // put your setup code here, to run once:
  serviceInit();
  Wire.begin(93);                // join i2c bus with address #93
  Wire.onReceive(receiveEvent); // register event

  while(1){
  if(buffer[2] == 0x0F){
    //SERVE!!!!!
    serviceShot();
    delay(5000);
    buffer[2] == 0x00;
    }
  }
}

#include <DynamixelSerial.h>

//Pins & Dynamixel ID config
#define ARM_ID 15
#define GRIP_ID 14
#define DYNA_CON 2
#define PNEU_PIN 12

//Constants
#define EXTEND 0
#define RETRACT 1
#define SER_DELAY 1000 //ms

//Positioning arm to service
void armDeploy()
{
  int angle = 150-120;
  int angle_tr = (double)angle * 1023 / 300;
  Dynamixel.move(ARM_ID,angle_tr);
}

//Positioning arm to initialize hitter
void armRetract()
{
  int angle = 150+120;
  int angle_tr = (double)angle * 1023 / 300;
  Dynamixel.move(ARM_ID,angle_tr);
}

//Dropping shuttlecock
void gripperOpen()
{
  int angle = 150 + 60;
  int angle_tr = (double)angle * 1023 / 300;
  Dynamixel.move(GRIP_ID,angle_tr);
}

//Close the gripper
void gripperClose()
{
  int angle = 150;
  int angle_tr = (double)angle * 1023 / 300;
  Dynamixel.move(GRIP_ID,angle_tr);
}

//Position the racket to bottom (ready for shooting)
void racketBottom()
{
  digitalWrite(PNEU_PIN,EXTEND);
}

//Shot the racket up!
void racketUp()
{
  digitalWrite(PNEU_PIN,RETRACT);
}

//Service routine
void serviceShot()
{
  armDeploy();
  delay(5000);
  
  gripperOpen();
  delay(SER_DELAY);
  
  racketUp();
  delay(500);
  
  racketBottom();
  gripperClose();
  armRetract();
}

//initialization needed
void serviceInit()
{
  Dynamixel.begin(1000000,2);
  Dynamixel.setEndless(ARM_ID,OFF);
  Dynamixel.setEndless(GRIP_ID,OFF);
  Dynamixel.setMaxTorque(ARM_ID,512);
  Dynamixel.setMaxTorque(GRIP_ID,512);
  Dynamixel.move(ARM_ID,240*1023/300);
  Dynamixel.move(GRIP_ID,150*1023/300);
  delay(1000);
  pinMode(PNEU_PIN,OUTPUT);
  digitalWrite(PNEU_PIN,EXTEND);
}

void setup()
{
  // put your setup code here, to run once:
  serviceInit();
}

void loop()
{
  // put your main code here, to run repeatedly:
  serviceShot();
  delay(5000);
}
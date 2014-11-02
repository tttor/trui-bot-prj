#include <arduino/Arduino.h>
<<<<<<< HEAD
#include <DynamixelSerial/DynamixelSerial.h>
#include <Wire.h>
=======
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/custom.h>
#include <actuator/motor.h>
>>>>>>> upstream/master

#define ARM_ID 15
#define GRIP_ID 14
#define DYNA_CON 2
#define PNEU_PIN 3

//Constants
#define RETRACT 0
#define EXTEND 1
#define SER_DELAY 210 //ms

<<<<<<< HEAD

=======
const int numerator= 24; // for slave with 13 ppr encoder
const int denominator= 5; // for slave with 13 ppr encoder
// const int numerator= 600; // for slave with 3 ppr encoder
// const int denominator= 29; // for slave with 3 ppr encoder


const int encoder_out_a_pin = 2;
const int encoder_out_b_pin = 3;
const int encoder_resolution = 360; //Only needed for Initialization, not used unless .rot() is called

int received_speed;
float speed;

trui::Motor motor(pwm_pin, dir_pin, encoder_out_a_pin, encoder_out_b_pin, encoder_resolution, outmax, outmin, numerator, denominator);
>>>>>>> upstream/master

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
  
<<<<<<< HEAD
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

        // start serial for output

  while(1){
  //if(buffer[2] == 0x0F)
    //SERVE!!!!!
    serviceShot();
    delay(5000);
  //  buffer[2] == 0x00;
  //}
  }
}
=======
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;      // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
 
  // set compare match register to desired timer count:
  OCR1A = 780; // 780 for 50ms
  // target time = timer resolution * (timer counts + 1)
  // (timer counts + 1) = target time / timer resolution
  // (timer counts + 1) = 50.10^-3 / (1 / (16.10^6 / 1024))

  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  interrupts();             // enable all interrupts
}

ISR(TIMER1_COMPA_vect)        // interrupt service routine 
{
  //interrupts();
  motor.set_speed(speed);
}

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  setup();
  motor.setup();

  Serial.begin(9600);

  long timeNow = 0, timeOld = 0;
  int incoming_byte = 0;
  uint8_t buffer[7];

  while (true) {

    //1. Wait msg from master

    while(true){
      if(Serial.available() >= 7) {
        buffer[0]= Serial.read();
        if (buffer[0] == 0xCE){
          for (int i=1; i<7; i++) {
            buffer[i]= Serial.read();
          }
          break;
        }
      }      
    }

    received_speed = (buffer[2] << 8);
    received_speed |= (buffer[1]);

    if (buffer[3] == 0x0C) speed= -1.0 * received_speed;
    else if (buffer[3] == 0xCC) speed= 1.0 * received_speed;
    // Serial.write(buffer[3]);

    // speed= (float) received_speed;

    // //2. Identify msg
    // //2A. If msgid = manual_setpoint, set speed control to cmd_speed
    // if ( (rx_msg.msgid==set_speed_msgid) ) {
    //   mavlink_manual_setpoint_t msg;
    //   mavlink_msg_manual_setpoint_decode(&rx_msg, &msg);
      
    //   speed= msg.roll;
    // } 
    // //2B. If msgid = asking for actual speed, send back actual speed to master
    // else if ( (rx_msg.msgid==actual_speed_query_msgid) ) {
    //   float actual_speed;
    //   mavlink_message_t msg_to_master;
    //   uint8_t system_id= MAV_TYPE_RBMT;
    //   uint8_t component_id= MAV_COMP_ID_ARDUINO_SLAVE1;

    //   uint32_t time_boot_ms= millis(); 
    //   mavlink_msg_manual_setpoint_pack(system_id, component_id, &msg_to_master, time_boot_ms, actual_speed, 0., 0., 0., 0, 0);

    //   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    //   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_to_master);

    //   Serial.write(buf, len);
    // }
    
  }

  return 0;
}
>>>>>>> upstream/master

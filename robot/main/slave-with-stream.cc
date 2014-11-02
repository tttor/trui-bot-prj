#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/custom.h>
#include <actuator/motor.h>

#define no_skip_char 1

int timer1_counter;
const size_t pwm_pin = 11;
const size_t dir_pin = 12; 
const float outmax = 100.0; 
const float outmin = -100.0; 

const int encoder_out_a_pin = 2;
const int encoder_out_b_pin = 3;
const int encoder_resolution = 360; //Only needed for Initialization, not used unless .rot() is called

uint32_t received_speed;
float speed;



trui::Motor motor(pwm_pin, dir_pin, encoder_out_a_pin, encoder_out_b_pin, encoder_resolution, outmax, outmin);

int timedPeek()
{
  int c;
  unsigned long _startMillis;
  unsigned long _timeout; 
  _startMillis = millis();
  do {
    c = Serial.peek();
    if (c >= 0) return c;
  } while(millis() - _startMillis < _timeout);
  return -1;     // -1 indicates timeout
}

int peekNextDigit()
{
  int c;
  while (1) {
    c = timedPeek();
    if (c < 0) return c;  // timeout
    if (c == '-') return c;
    if (c >= '0' && c <= '9') return c;
    Serial.read();  // discard non-numeric
  }
}

float parseFloat(char skipChar){
  boolean isNegative = false;
  boolean isFraction = false;
  long value = 0;
  char c;
  float fraction = 1.0;

  c = peekNextDigit();
    // ignore non numeric leading characters
  if(c < 0)
    return 0; // zero returned if timeout

  do{
    if(c == skipChar)
      ; // ignore
    else if(c == '-')
      isNegative = true;
    else if (c == '.')
      isFraction = true;
    else if(c >= '0' && c <= '9')  {      // is c a digit?
      value = value * 10 + c - '0';
      if(isFraction)
         fraction *= 0.1;
    }
    Serial.read();  // consume the character we got with peek
    c = timedPeek();
  }
  while( (c >= '0' && c <= '9')  || c == '.' || c == skipChar );

  if(isNegative)
    value = -value;
  if(isFraction)
    return value * fraction;
  else
    return value;
}

void setup()
{
  
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;      // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
 
  // set compare match register to desired timer count:
  OCR1A = 780;
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

void receive_data() {

}

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  //setup();
  motor.setup();

  Serial.begin(9600);
  speed = 0;
  long timeNow = 0, timeOld = 0;
  int incoming_byte = 0;
  uint8_t buffer[7];

  while (true) {

    //1. Wait msg from master


       if (Serial.available() > 0) {
       buffer[0] = Serial.read();//parseFloat(no_skip_char);
      //   if(buffer[0] == 206){
      // buffer[1] = Serial.read();
      // buffer[2] = Serial.read();
      // buffer[3] = Serial.read();
      // buffer[4] = Serial.read();
      // buffer[5] = Serial.read();
      // buffer[6] = Serial.read();
      // //buffer[7] = Serial.read();
      // //buffer[8] = Serial.read();
      //   }
       }

     // received_speed = (buffer[4] << 24);
     // received_speed |= (buffer[3] << 16);
     // received_speed |= (buffer[2] << 8);
     // received_speed |= (buffer[1]);
       received_speed = buffer[0];
      
    // if(buffer[5] == 0x00 || buffer[5] == 0x0C || buffer[5] == 0xCC) 
       speed = (float)received_speed;

     //speed = 50;//(float) received_speed;
     timeNow = millis();
      if(timeNow - timeOld >= 50){
      timeOld = timeNow;
      motor.set_speed(speed);
    }

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

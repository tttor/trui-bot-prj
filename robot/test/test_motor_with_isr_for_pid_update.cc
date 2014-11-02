#include <arduino/Arduino.h>
#include <mavlink/v1.0/common/mavlink.h>
#include <comm/custom.h>
#include <actuator/motor.h>

//
void setup_timer1();
ISR(TIMER1_COMPA_vect);

//
const size_t pwm_pin = 11;
const size_t dir_pin = 12; 
const float outmax = 100.0; 
const float outmin = -100.0; 

const int encoder_out_a_pin = 2;
const int encoder_out_b_pin = 3;
const int encoder_resolution = 360; //Only needed for Initialization, not used unless .rot() is called
  
trui::Motor g_motor(pwm_pin, dir_pin, encoder_out_a_pin, encoder_out_b_pin, encoder_resolution, outmax, outmin);

//
static float g_cmd_speed = 0;

int main() {
  init();
  Serial.begin(9600);
  pinMode(13,OUTPUT);
  Serial1.begin(9600);
  setup_timer1();
  
  bool led = false;
  while (1) {
    const uint8_t channel = MAVLINK_COMM_1;
    const uint8_t msgid = MAVLINK_MSG_ID_ATTITUDE;

    mavlink_message_t rx_msg;
    mavlink_status_t rx_status;

    bool msg_found = false;
    while (!msg_found) {
      if (Serial1.available() > 0) {
        if (mavlink_parse_char(channel, Serial1.read(), &rx_msg, &rx_status)) {
          if ( (rx_msg.msgid==msgid) ) {
            break;
          }
        }
      }
    }// while (!msg_found)

    //
    mavlink_attitude_t msg;
    mavlink_msg_attitude_decode(&rx_msg, &msg);

    //
    g_cmd_speed = msg.rollspeed;

    //
    // Serial.println("----------------------------------");
    // Serial.print("msg.roll= "); Serial.println(msg.roll);
    // Serial.print("msg.pitch= "); Serial.println(msg.pitch);
    // Serial.print("msg.yaw= "); Serial.println(msg.yaw);
    // Serial.print("msg.rollspeed= "); Serial.println(msg.rollspeed);
    // Serial.print("msg.pitchspeed= "); Serial.println(msg.pitchspeed);
    // Serial.print("msg.yawspeed= "); Serial.println(msg.yawspeed);

    led = !led;
    digitalWrite(13,led);
    // delay(1);
  }

  return 0;
}

void setup_timer1() {
  // initialize timer1 
  noInterrupts();// disable all interrupts

  TCCR1A = 0;      // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
 
  // set compare match register to desired timer count:
  OCR1A = 780;// TODO @tttor: why this value?

  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  
  interrupts();// enable all interrupts
}

ISR(TIMER1_COMPA_vect) {
  //interrupts();// for the external interrupt (for encoders) to interrupt
  g_motor.set_speed(g_cmd_speed);
}

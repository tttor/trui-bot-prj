#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/custom.h>
#include <actuator/motor.h>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  
  Serial.begin(57600);

  const size_t pwm_pin = 11;
  const size_t dir_pin = 12; 
  const float outmax = 100.0; 
  const float outmin = -100.0; 

  const int encoder_out_a_pin = 2;
  const int encoder_out_b_pin = 3;
  const int encoder_resolution = 360; //Only needed for Initialization, not used unless .rot() is called
  
  trui::Motor motor(pwm_pin, dir_pin, encoder_out_a_pin, encoder_out_b_pin, encoder_resolution, outmax, outmin);
  motor.setup();

  long timeNow = 0, timeOld = 0;
  float x;

  while (true) {
    //1. Wait msg from master
    const uint8_t channel = MAVLINK_COMM_0;
    const uint8_t set_speed_msgid = MAVLINK_MSG_ID_MANUAL_SETPOINT;
    const uint8_t actual_speed_query_msgid = MAVLINK_MSG_ID_COMMAND_INT;
    
    mavlink_message_t rx_msg;
    mavlink_status_t rx_status;
    
    while (true) {
      if (Serial.available() > 0) {
        if (mavlink_parse_char(channel, Serial.read(), &rx_msg, &rx_status)) {
          if ( (rx_msg.msgid==set_speed_msgid) ) {
            break;
          } 
          else if ( (rx_msg.msgid==actual_speed_query_msgid) ) {
            break;
          }
        }
      }
    }

    //2. Identify msg
    //2A. If msgid = manual_setpoint, set speed control to cmd_speed
    if ( (rx_msg.msgid==set_speed_msgid) ) {
      mavlink_manual_setpoint_t msg;
      mavlink_msg_manual_setpoint_decode(&rx_msg, &msg);
      float actual_speed= msg.roll;
      x = motor.set_speed(actual_speed);
    } 
    //2B. If msgid = asking for actual speed, send back actual speed to master
    else if ( (rx_msg.msgid==actual_speed_query_msgid) ) {
      float actual_speed;
      mavlink_message_t msg_to_master;
      uint8_t system_id= MAV_TYPE_RBMT;
      uint8_t component_id= MAV_COMP_ID_ARDUINO_SLAVE1;

      uint32_t time_boot_ms= millis(); 
      mavlink_msg_manual_setpoint_pack(system_id, component_id, &msg_to_master, time_boot_ms, actual_speed, 0., 0., 0., 0, 0);

      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_to_master);

      Serial.write(buf, len);
    }
    
  }

  return 0;
}

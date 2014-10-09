#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/custom.h>

int main() {
  init();

  const uint8_t channel1= MAVLINK_COMM_1;
  const uint8_t channel2= MAVLINK_COMM_2;
  const uint8_t channel3= MAVLINK_COMM_3;
  const uint8_t desired_msgid= MAVLINK_MSG_ID_ATTITUDE;    

  mavlink_message_t rx_msg1;
  mavlink_status_t rx_status1;

  mavlink_message_t rx_msg2;
  mavlink_status_t rx_status2;

  mavlink_message_t rx_msg3;
  mavlink_status_t rx_status3;
  // uint32_t last_time;
  // uint32_t duration;
  
  Serial.begin(115200);
  Serial1.begin(57600);
  Serial2.begin(57600);    
  Serial3.begin(57600);

  //
  const uint16_t rate = 10;
  while (true) {
    
    // last_time= millis();
    while (true) {
      if (Serial1.available() > 0) {
        if (mavlink_parse_char(channel1, Serial1.read(), &rx_msg1, &rx_status1)) {
          if ( (rx_msg1.msgid==desired_msgid) ) {
            break;
          }
        }
      }
    }
    while (true) {
      if (Serial2.available() > 0) {
        if (mavlink_parse_char(channel2, Serial2.read(), &rx_msg2, &rx_status2)) {
          if ( (rx_msg2.msgid==desired_msgid) ) {
            break;
          }
        }
      }
    }
    while (true) {
      if (Serial3.available() > 0) {
        if (mavlink_parse_char(channel3, Serial3.read(), &rx_msg3, &rx_status3)) {
          if ( (rx_msg3.msgid==desired_msgid) ) {
            break;
          }
        }
      }
    }
    // duration= millis() - last_time;

    mavlink_attitude_t msg1;
    mavlink_attitude_t msg2;
    mavlink_attitude_t msg3;
    mavlink_msg_attitude_decode(&rx_msg1, &msg1);
    mavlink_msg_attitude_decode(&rx_msg2, &msg2);
    mavlink_msg_attitude_decode(&rx_msg3, &msg3);

    Serial.print("msg1.rollspeed= "); Serial.println(msg1.rollspeed); 
    Serial.print("msg2.rollspeed= "); Serial.println(msg2.rollspeed);
    Serial.print("msg3.rollspeed= "); Serial.println(msg3.rollspeed);
    //Serial.print(", "); Serial.println(duration);

    delay(1000/rate);

  }

  return 0;
}


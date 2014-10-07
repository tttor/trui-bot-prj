#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/custom.h>

int main() {
  init();

  const uint8_t channel= MAVLINK_COMM_2;
  const uint8_t desired_msgid= MAVLINK_MSG_ID_ATTITUDE;  

  mavlink_message_t rx_msg;
  mavlink_status_t rx_status;
  
  Serial.begin(115200);
  Serial2.begin(57600);    

  //
  const uint16_t rate = 10;
  while (true) {
  
    while (true) {
      if (Serial2.available() > 0) {
        if (mavlink_parse_char(channel, Serial2.read(), &rx_msg, &rx_status)) {
          if ( (rx_msg.msgid==desired_msgid) ) {
            break;
          }
        }
      }
    }

    mavlink_attitude_t msg;
    mavlink_msg_attitude_decode(&rx_msg, &msg);

    Serial.print("msg.rollspeed= "); Serial.println(msg.rollspeed);

    delay(1000/rate);

  }

  return 0;
}


#include <arduino/Arduino.h>
#include <mavlink/v1.0/common/mavlink.h>
#include <comm/custom.h>

int main() {
  init();
  Serial.begin(9600);
  Serial1.begin(9600);
  
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

    mavlink_attitude_t msg;
    mavlink_msg_attitude_decode(&rx_msg, &msg);

    Serial.println("----------------------------------");
    Serial.print("msg.roll= "); Serial.println(msg.roll);
    Serial.print("msg.pitch= "); Serial.println(msg.pitch);
    Serial.print("msg.yaw= "); Serial.println(msg.yaw);
    Serial.print("msg.rollspeed= "); Serial.println(msg.rollspeed);
    Serial.print("msg.pitchspeed= "); Serial.println(msg.pitchspeed);
    Serial.print("msg.yawspeed= "); Serial.println(msg.yawspeed);

    delay(100);
  }

  return 0;
}


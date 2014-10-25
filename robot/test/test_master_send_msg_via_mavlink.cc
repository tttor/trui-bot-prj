#include <arduino/Arduino.h>
#include <mavlink/v1.0/common/mavlink.h>
#include <comm/custom.h>

int main() {
  init();// needs to be called before everthing, otherwise some functions won't work
  Serial.begin(9600);

  const uint8_t channel= MAVLINK_COMM_0;
  const uint8_t msgid= MAVLINK_MSG_ID_ATTITUDE;   
  uint8_t system_id= MAV_TYPE_RBMT;
  uint8_t component_id= MAV_COMP_ID_ARDUINO_MASTER;

  while (true) {
    mavlink_message_t msg;

    float roll, pitch, yaw;
    roll = 1.111;
    pitch = 2.222;
    yaw = 3.333;

    float vroll, vpitch, vyaw;
    vroll = 4.444;
    vpitch = 5.555;
    vyaw = 6.666;

    mavlink_msg_attitude_pack(system_id, component_id, &msg, millis(), 
                              roll, pitch, yaw, 
                              vroll, vpitch, vyaw);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);    
    
    Serial.write(buf, len); 

    delay(100);
  }

  return 0;
}


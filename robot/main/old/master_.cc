#include <arduino/Arduino.h>

int main() {
  init();// needs to be called before everthing, otherwise some functions won't work
  

  // // TODO @tttor: fix problem if using packet_handler: probably due to using the main "Serial", 
  // // Build up a mavlink system _of_ this board
  // mavlink_system_t mavlink_system;
  // mavlink_system.sysid = 1;
  // mavlink_system.compid = 1;
  // trui::ArduinoMavlinkPacketHandler packet_handler(mavlink_system, "Serial", 57600);

  // //
  // uint8_t desired_sender_sysid = 1;
  // uint8_t desired_sender_compid = 240;
  // uint8_t desired_msgid = MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
  // mavlink_set_position_target_local_ned_t msg;
  
  // Serial.begin(57600);
  // packet_handler.wait(desired_sender_sysid, desired_sender_compid, desired_msgid, &msg);// will block 
  
  //  
  Serial.begin(57600);
  const uint8_t channel = MAVLINK_COMM_0;
  
  const uint8_t desired_msgid = MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
  
  mavlink_message_t rx_msg;
  mavlink_status_t rx_status;
  
  while (true) {
    if (Serial.available() > 0) {
      if (mavlink_parse_char(channel, Serial.read(), &rx_msg, &rx_status)) {
        if ( (rx_msg.msgid==desired_msgid) ) {
          break;
        }
      }
    }
  }

  mavlink_set_position_target_local_ned_t msg;
  mavlink_msg_set_position_target_local_ned_decode(&rx_msg, &msg);

  //
  const uint16_t rate = 1;
  while (true) {
    Serial.println("found");

    Serial.print("msg.x= "); Serial.println(msg.x);
    Serial.print("msg.y= "); Serial.println(msg.y);
    Serial.print("msg.z= "); Serial.println(msg.z);
    Serial.print("msg.vx= "); Serial.println(msg.vx);
    Serial.print("msg.vy= "); Serial.println(msg.vy);
    Serial.print("msg.vz= "); Serial.println(msg.vz);
    Serial.print("msg.yaw_rate= "); Serial.println(msg.yaw_rate);
    // Serial.println(msg.target_system);
    // Serial.println(msg.target_component);

    delay(1000/rate);
  }

  return 0;
}

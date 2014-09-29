#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  pinMode(13,OUTPUT);
  
  mavlink_system_t mavlink_system;
  mavlink_system.sysid = MAV_TYPE_RBMT;
  mavlink_system.compid = MAV_COMP_ID_ARDUINO_SLAVE2;

  const size_t baudrate_master_to_slave = 9600;  
  const unsigned int kDuration = 100;

  crim::ArduinoMavlinkPacketHandler packet_handler(mavlink_system, "Serial", baudrate_master_to_slave);
  
  uint32_t loop_counter = 0;

  digitalWrite(13,HIGH);

  while (true) {
    uint8_t sysid = MAV_TYPE_RBMT;   ///< ID of message sender system/aircraft
    uint8_t compid = MAV_COMP_ID_ARDUINO_MASTER;  ///< ID of the message sender component
    uint8_t msgid = MAVLINK_MSG_ID_ATTITUDE;   ///< ID of message in payload
    mavlink_attitude_t att_msg;
    
    packet_handler.wait(sysid, compid, msgid, &att_msg);
    digitalWrite(13,LOW);
    
    if(att_msg.rollspeed < 75.0) {
      digitalWrite(13,LOW);
      delay(kDuration);
    } else {
      digitalWrite(13,HIGH);
      delay(kDuration);
    }
    
    ++loop_counter;
  }
  
  Serial.end();
  return 0;
}

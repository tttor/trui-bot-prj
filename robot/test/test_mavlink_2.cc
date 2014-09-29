#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  mavlink_system_t mavlink_system;
  mavlink_system.sysid = MAV_TYPE_TEST_BENCH;
  mavlink_system.compid = MAV_COMP_ID_STATIC_TEST_BENCH_ARDUINO;
  
  crim::ArduinoMavlinkPacketHandler packet_handler(mavlink_system, "Serial", 9600);
  
  uint32_t loop_counter = 0;
  while (true) {
    uint8_t sysid = MAV_TYPE_TEST_BENCH;   ///< ID of message sender system/aircraft
    uint8_t compid = MAV_COMP_ID_STATIC_TEST_BENCH_ARDUINO;  ///< ID of the message sender component
  	uint8_t msgid = MAVLINK_MSG_ID_ATTITUDE;   ///< ID of message in payload
    mavlink_attitude_t att_msg;
    
    packet_handler.wait(sysid, compid, msgid, &att_msg);
    
    Serial.println("---------------------------------------------------------");
    Serial.print("att_msg.roll= "); Serial.println(att_msg.roll);
    Serial.print("att_msg.pitch= "); Serial.println(att_msg.pitch);
    Serial.print("att_msg.yaw= "); Serial.println(att_msg.yaw);
    Serial.print("att_msg.rollspeed= "); Serial.println(att_msg.rollspeed);
    Serial.print("att_msg.pitchspeed= "); Serial.println(att_msg.pitchspeed);
    Serial.print("att_msg.yawspeed= "); Serial.println(att_msg.yawspeed);
    
    ++loop_counter;
    Serial.print("loop_counter= "); Serial.println(loop_counter);
    delay(100);
  }
  
  Serial.end();
  return 0;
}

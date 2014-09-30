#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  
  mavlink_system_t mavlink_system;
  mavlink_system.sysid = MAV_TYPE_TEST_BENCH;
  mavlink_system.compid = MAV_COMP_ID_STATIC_TEST_BENCH_ARDUINO;

  const size_t baudrate_pc2master = 57600;
  trui::ArduinoMavlinkPacketHandler packet_handler(mavlink_system, "Serial", 9600);

  uint8_t sender_sysid = 1;
  uint8_t sender_compid = 2;
  uint8_t desired_msgid;
 
  const uint16_t rate = 10;
  while (true) {
    msgid = MAVLINK_MSG_ID_ATTITUDE;
    
    mavlink_attitude_t att_msg;
    packet_handler.wait(sender_sysid, sender_compid, sent_msgid, &att_msg);

    delay(1000/rate);
  }
  
  return 0;
}

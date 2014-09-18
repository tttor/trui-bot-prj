#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>

#define TEST_SENDING_ATTITUDE_MSG

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  mavlink_system_t mavlink_system;
  mavlink_system.sysid = MAV_TYPE_TEST_BENCH;
  mavlink_system.compid = MAV_COMP_ID_STATIC_TEST_BENCH_ARDUINO;

  //
  trui::ArduinoMavlinkPacketHandler packet_handler(mavlink_system, "Serial3", 9600);

  while (true) {
#ifdef TEST_SENDING_ATTITUDE_MSG  
     // Define the attitude msg
    mavlink_attitude_t msg;
  
    msg.time_boot_ms = millis();
    msg.roll = 0.;
    msg.pitch = 0.;
    msg.yaw = 0.;
    msg.rollspeed = 67.890;
    msg.pitchspeed = 43.542;
    msg.yawspeed = 27.344;
#endif

    packet_handler.wrap(msg);
    packet_handler.send();
    
    delay(500);
  }
  
  Serial.end();
  return 0;
}

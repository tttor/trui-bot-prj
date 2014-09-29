#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);  

  mavlink_system_t mavlink_system;
  mavlink_system.sysid = MAV_TYPE_RBMT;
  mavlink_system.compid = MAV_COMP_ID_ARDUINO_MASTER;

  const size_t baudrate_master_to_slave = 9600;

  crim::ArduinoMavlinkPacketHandler packet_handler_slave1(mavlink_system, "Serial1", baudrate_master_to_slave);
  crim::ArduinoMavlinkPacketHandler packet_handler_slave2(mavlink_system, "Serial2", baudrate_master_to_slave);
  crim::ArduinoMavlinkPacketHandler packet_handler_slave3(mavlink_system, "Serial3", baudrate_master_to_slave);
  
  while (true) {

  //  Serial.print("Hello");
    mavlink_attitude_t msg;
  
    msg.time_boot_ms = millis();
    msg.rollspeed = 50.;
    packet_handler_slave2.wrap(msg);
    packet_handler_slave2.send();    
    
    delay(500);

    msg.time_boot_ms = millis();
    msg.rollspeed = 100.;
    packet_handler_slave2.wrap(msg);
    packet_handler_slave2.send();    
    
    delay(500);
  }
  
  Serial.end();
  return 0;
}

#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/custom.h>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  
  Serial.begin(57600);
  

  // const uint8_t channel = MAVLINK_COMM_0;
  // const uint8_t desired_msgid = MAVLINK_MSG_ID_ATTITUDE;
  
  // mavlink_message_t rx_msg;
  // mavlink_status_t rx_status;
  
  // while (true) {
  //   if (Serial.available() > 0) {
  //     if (mavlink_parse_char(channel, Serial.read(), &rx_msg, &rx_status)) {
  //       if ( (rx_msg.msgid==desired_msgid) ) {
  //         break;
  //       }
  //     }
  //   }
  // }

  // mavlink_attitude_t msg;
  // mavlink_msg_attitude_decode(&rx_msg, &msg);

  //
  const uint16_t rate = 1;
  while (true) {
    mavlink_message_t msg2;
    uint8_t system_id= MAV_TYPE_RBMT;
    uint8_t component_id= MAV_COMP_ID_ARDUINO_MASTER;
    uint32_t time_boot_ms= millis();

    mavlink_msg_attitude_pack(system_id, component_id, &msg2, time_boot_ms, 0., 0., 0., 23.0, 0., 0.);

    // Copy the message to the send buffer
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg2);
	  
    // Send the message with the standard UART send function
    // uart0_send might be named differently depending on
    // the individual microcontroller / library in use.
    Serial.write(buf, len);

    delay(1000/rate);
  }

  return 0;
}

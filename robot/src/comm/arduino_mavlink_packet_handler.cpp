#include "arduino_mavlink_packet_handler.hpp"

using namespace trui;

ArduinoMavlinkPacketHandler::ArduinoMavlinkPacketHandler(mavlink_system_t mavlink_system, String port, uint32_t baud_rate)
    : mavlink_system_(mavlink_system), port_(port), baud_rate_(baud_rate), msg_(new mavlink_message_t()) {
    Serial.begin(baud_rate_);
}

ArduinoMavlinkPacketHandler::~ArduinoMavlinkPacketHandler() {
  Serial.end();

  delete msg_;
}

void ArduinoMavlinkPacketHandler::send() {
  // Copy the message to the send buffer
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg_);
  
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  if (port_ == "Serial")
    Serial.write(buf, len);
}

void ArduinoMavlinkPacketHandler::wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_message_t* msg) {
  mavlink_message_t rx_msg;
  mavlink_status_t rx_status;
  
  const uint8_t channel = MAVLINK_COMM_0;
  Serial.println("hhoaoaaooa");
  while (true) {
    if (Serial.available() > 0) {
      if (mavlink_parse_char(channel, Serial.read(), &rx_msg, &rx_status)) {
        if ( (rx_msg.msgid==msgid) ) {
          break;
        }
      }
    }
  }

  // bool desired_msg_found = false;
  
  // // This loop below should be run at its possible highest rate
  // while (!desired_msg_found) {
  //   mavlink_message_t rx_msg;
  //   mavlink_status_t rx_status;
    
  //   int available;
  //   if (port_ == "Serial")
  //     available = Serial.available();

  //   if (available > 0) {
  //     uint8_t channel;
  //     uint8_t c;
      
  //     if (port_ == "Serial") {
  //       c = Serial.read();
  //       channel = MAVLINK_COMM_0;
  //     }
      
  //     if(mavlink_parse_char(channel, c, &rx_msg, &rx_status)) {
  //        if ( (rx_msg.msgid==msgid) /* and(rx_msg.sysid==sysid) and (rx_msg.compid=compid) */ ) {
  //         desired_msg_found = true;
  //         *msg = rx_msg;
  //        }
  //     }
  //   }// if (available > 0)
  // }// while (!desired_msg_found)
}

void ArduinoMavlinkPacketHandler::wrap(mavlink_attitude_t raw_msg) {
  mavlink_msg_attitude_pack(mavlink_system_.sysid, mavlink_system_.compid, msg_, 
                            raw_msg.time_boot_ms, 
                            raw_msg.roll,raw_msg.pitch,raw_msg.yaw, 
                            raw_msg.rollspeed,raw_msg.pitchspeed,raw_msg.yawspeed);
}
                   
void ArduinoMavlinkPacketHandler::wrap(mavlink_attitude_quaternion_t raw_msg) {
  mavlink_msg_attitude_quaternion_pack(mavlink_system_.sysid, mavlink_system_.compid, msg_,
                                       raw_msg.time_boot_ms,
                                       raw_msg.q1, raw_msg.q2, raw_msg.q3, raw_msg.q4,
                                       raw_msg.rollspeed, raw_msg.pitchspeed, raw_msg.yawspeed);
}

void ArduinoMavlinkPacketHandler::wrap(mavlink_global_position_int_t raw_msg) {
  mavlink_msg_global_position_int_pack(mavlink_system_.sysid, mavlink_system_.compid, msg_,
                                       raw_msg.time_boot_ms,
                                       raw_msg.lat, raw_msg.lon,raw_msg.alt,
                                       raw_msg.relative_alt, raw_msg.vx, raw_msg.vy, raw_msg.vz, raw_msg.hdg);
}
                   
void ArduinoMavlinkPacketHandler::wrap(mavlink_mission_set_current_t raw_msg) {
  mavlink_msg_mission_set_current_pack(mavlink_system_.sysid, mavlink_system_.compid, msg_,
                                       raw_msg.target_system, raw_msg.target_component, raw_msg.seq);
}

void ArduinoMavlinkPacketHandler::wrap(mavlink_rc_channels_raw_t raw_msg) {
  mavlink_msg_rc_channels_raw_pack(mavlink_system_.sysid, mavlink_system_.compid, msg_, 
                                   raw_msg.time_boot_ms,
                                   raw_msg.port,
                                   raw_msg.chan1_raw, raw_msg.chan2_raw, raw_msg.chan3_raw, raw_msg.chan4_raw, 
                                   raw_msg.chan5_raw, raw_msg.chan6_raw, raw_msg.chan7_raw, raw_msg.chan8_raw, 
                                   raw_msg.rssi);
}

void ArduinoMavlinkPacketHandler::wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_attitude_t* the_msg){
  wait(sysid, compid, msgid, msg_); // block till receive the desired msg
  mavlink_msg_attitude_decode(msg_, the_msg);
}

void ArduinoMavlinkPacketHandler::wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_attitude_quaternion_t* the_msg) {
  wait(sysid, compid, msgid, msg_); // block till receive the desired msg
  mavlink_msg_attitude_quaternion_decode(msg_, the_msg);
}

void ArduinoMavlinkPacketHandler::wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_global_position_int_t* the_msg) {
  wait(sysid, compid, msgid, msg_); // block till receive the desired msg
  mavlink_msg_global_position_int_decode(msg_, the_msg);
}

void ArduinoMavlinkPacketHandler::wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_mission_set_current_t* the_msg) {
  wait(sysid, compid, msgid, msg_); // block till receive the desired msg
  mavlink_msg_mission_set_current_decode(msg_, the_msg);
}

void ArduinoMavlinkPacketHandler::wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_rc_channels_raw_t* the_msg) {
  wait(sysid, compid, msgid, msg_); // block till receive the desired msg
  mavlink_msg_rc_channels_raw_decode(msg_, the_msg);
}

void ArduinoMavlinkPacketHandler::wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_set_position_target_local_ned_t* the_msg) {
  wait(sysid, compid, msgid, msg_); // block till receive the desired msg
  mavlink_msg_set_position_target_local_ned_decode(msg_, the_msg);
}
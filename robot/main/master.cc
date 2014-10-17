#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/custom.h>

void isr() {
  const uint8_t channel1= MAVLINK_COMM_1;
  const uint8_t channel2= MAVLINK_COMM_2;
  const uint8_t channel3= MAVLINK_COMM_3;
  const uint8_t desired_msgid= MAVLINK_MSG_ID_MANUAL_SETPOINT;

  mavlink_message_t rx_msg1;
  mavlink_status_t rx_status1;

  mavlink_message_t rx_msg2;
  mavlink_status_t rx_status2;

  mavlink_message_t rx_msg3;
  mavlink_status_t rx_status3;

  //6A. 1. Send next speed command to all motors, :stop

  float slave1_cmd_speed= 0.;
  float slave2_cmd_speed= 0.;
  float slave3_cmd_speed= 0.;

  mavlink_message_t msg_to_slave1;
  mavlink_message_t msg_to_slave2;
  mavlink_message_t msg_to_slave3;
  uint8_t system_id= MAV_TYPE_RBMT;
  uint8_t component_id= MAV_COMP_ID_ARDUINO_MASTER;
    
  uint32_t time_boot_ms= millis(); 
  mavlink_msg_manual_setpoint_pack(system_id, component_id, &msg_to_slave1, time_boot_ms, slave1_cmd_speed, 0., 0., 0., 0, 0);
  mavlink_msg_manual_setpoint_pack(system_id, component_id, &msg_to_slave2, time_boot_ms, slave2_cmd_speed, 0., 0., 0., 0, 0);
  mavlink_msg_manual_setpoint_pack(system_id, component_id, &msg_to_slave3, time_boot_ms, slave3_cmd_speed, 0., 0., 0., 0, 0);

  uint8_t buf1[MAVLINK_MAX_PACKET_LEN];
  uint8_t buf2[MAVLINK_MAX_PACKET_LEN];
  uint8_t buf3[MAVLINK_MAX_PACKET_LEN];
  uint16_t len1 = mavlink_msg_to_send_buffer(buf1, &msg_to_slave1);    
  uint16_t len2 = mavlink_msg_to_send_buffer(buf2, &msg_to_slave2);    
  uint16_t len3 = mavlink_msg_to_send_buffer(buf3, &msg_to_slave3);
    
  Serial1.write(buf1, len1);    
  Serial2.write(buf2, len2); 
  Serial3.write(buf3, len3);     
    
  //6B. Query actual speed for each motor for past cmd_speed
  time_boot_ms= millis(); 
  mavlink_msg_command_int_pack(system_id, component_id, &msg_to_slave1, 0, 0, 0, 0, 0, 0, 0., 0., 0., 0., 0, 0, 0.);
  len1 = mavlink_msg_to_send_buffer(buf1, &msg_to_slave1); 
  Serial1.write(buf1, len1);
  while (true) {
    if (Serial1.available() > 0) {
      if (mavlink_parse_char(channel1, Serial1.read(), &rx_msg1, &rx_status1)) {
        if ( (rx_msg1.msgid==desired_msgid) ) {
          break;
        }
      }
    }
  }

  time_boot_ms= millis(); 
  mavlink_msg_command_int_pack(system_id, component_id, &msg_to_slave2, 0, 0, 0, 0, 0, 0, 0., 0., 0., 0., 0, 0, 0.);
  len2 = mavlink_msg_to_send_buffer(buf2, &msg_to_slave2); 
  Serial2.write(buf2, len2);
  while (true) {
    if (Serial2.available() > 0) {
      if (mavlink_parse_char(channel2, Serial2.read(), &rx_msg2, &rx_status2)) {
        if ( (rx_msg2.msgid==desired_msgid) ) {
          break;
        }
      }
    }
  }

  time_boot_ms= millis(); 
  mavlink_msg_command_int_pack(system_id, component_id, &msg_to_slave3, 0, 0, 0, 0, 0, 0, 0., 0., 0., 0., 0, 0, 0.);
  len3 = mavlink_msg_to_send_buffer(buf3, &msg_to_slave3); 
  Serial3.write(buf3, len3);
  while (true) {
    if (Serial3.available() > 0) {
      if (mavlink_parse_char(channel3, Serial3.read(), &rx_msg3, &rx_status3)) {
        if ( (rx_msg3.msgid==desired_msgid) ) {
          break;
        }
      }
    }
  }

  mavlink_manual_setpoint_t msg;

  mavlink_msg_manual_setpoint_decode(&rx_msg1, &msg);
  float slave1_actual_speed= msg.roll;
  mavlink_msg_manual_setpoint_decode(&rx_msg2, &msg);
  float slave2_actual_speed= msg.roll;
  mavlink_msg_manual_setpoint_decode(&rx_msg3, &msg);
  float slave3_actual_speed= msg.roll;

  //6C. Send actual speed to PC controller
  mavlink_message_t msg_to_controller;
  
  time_boot_ms= millis(); 
  mavlink_msg_set_position_target_local_ned_pack(system_id, component_id, &msg_to_controller, 
    time_boot_ms, 0, 0, 0, 0, 0., 0., 0., slave1_actual_speed, slave2_actual_speed, slave3_actual_speed, 0., 0., 0., 0., 0.);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_to_controller); 
  Serial.write(buf, len);
}

int main() {
  init();
  Serial.begin(115200);
  Serial1.begin(57600);
  Serial2.begin(57600);    
  Serial3.begin(57600);

  const uint8_t channel= MAVLINK_COMM_0;
  const uint8_t desired_msgid= MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;    

  mavlink_message_t rx_msg;
  mavlink_status_t rx_status;  

  while (true) {
    
    // //1. wait for cmd_speed from serial from controller
    // while (true) {
    //   if (Serial.available() > 0) {
    //     if (mavlink_parse_char(channel, Serial.read(), &rx_msg, &rx_status)) {
    //       if ( (rx_msg.msgid==desired_msgid) ) {
    //         break;
    //       }
    //     }
    //   }
    // }

    // //2. break down the speed into the speed for 3 motors
    // mavlink_set_position_target_local_ned_t msg;
    // mavlink_msg_set_position_target_local_ned_decode(&rx_msg, &msg);

    // float slave1_cmd_speed= msg.vx;
    // float slave2_cmd_speed= msg.vy;
    // float slave3_cmd_speed= msg.vz;

    float slave2_cmd_speed= 50;

    //3. dispatch and send the 3 speed values
    // mavlink_message_t msg_to_slave1;
    mavlink_message_t msg_to_slave2;
    // mavlink_message_t msg_to_slave3;
    uint8_t system_id= MAV_TYPE_RBMT;
    uint8_t component_id= MAV_COMP_ID_ARDUINO_MASTER;    
    uint32_t time_boot_ms= millis(); 

    // mavlink_msg_manual_setpoint_pack(system_id, component_id, &msg_to_slave1, time_boot_ms, slave1_cmd_speed, 0., 0., 0., 0, 0);
    mavlink_msg_manual_setpoint_pack(system_id, component_id, &msg_to_slave2, time_boot_ms, slave2_cmd_speed, 0., 0., 0., 0, 0);
    // mavlink_msg_manual_setpoint_pack(system_id, component_id, &msg_to_slave3, time_boot_ms, slave3_cmd_speed, 0., 0., 0., 0, 0);

    // uint8_t buf1[MAVLINK_MAX_PACKET_LEN];
    uint8_t buf2[MAVLINK_MAX_PACKET_LEN];
    // uint8_t buf3[MAVLINK_MAX_PACKET_LEN];
    // uint16_t len1 = mavlink_msg_to_send_buffer(buf1, &msg_to_slave1);    
    uint16_t len2 = mavlink_msg_to_send_buffer(buf2, &msg_to_slave2);    
    // uint16_t len3 = mavlink_msg_to_send_buffer(buf3, &msg_to_slave3);
    
    // Serial1.write(buf1, len1);    
    Serial2.write(buf2, len2); 
    // Serial3.write(buf3, len3);     
    
    //4. activate timer according to dt using timer overflow interrupt
    //activateTimer();

    //5. back to 1
    //6. if timer overflow interrupt happens, do 6A
    
  }

  return 0;
}


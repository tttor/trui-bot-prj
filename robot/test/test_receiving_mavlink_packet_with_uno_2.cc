#include <arduino/Arduino.h>
#include <mavlink/v1.0/common/mavlink.h>
#include <comm/custom.h>

int main() {
  init();
  Serial.begin(57600);
  
  long unsigned int ctr = 0;
  while (1) {
    //
    // /////////////////////////////////////////////////////////////////////
    // // Serial.begin(57600);
    // const uint8_t channel = MAVLINK_COMM_0;
    // const uint8_t desired_msgid =  MAVLINK_MSG_ID_ATTITUDE;

    // mavlink_message_t rx_msg;
    // mavlink_status_t rx_status;

    // Serial.println("Waiting ...");
    // while (true) {
    //   if (Serial.available() > 0) {
    //     if (mavlink_parse_char(channel, Serial.read(), &rx_msg, &rx_status)) {
    //       if ( (rx_msg.msgid==desired_msgid) ) {
    //         break;
    //       }
    //     }
    //   }
    // }
    // ///////////////////////////////////////////////////////////////////////
    // bool desired_msg_found = false;
    // String port_ = "Serial";
    // mavlink_message_t* msg;
    // uint8_t msgid = MAVLINK_MSG_ID_ATTITUDE;
  
    // // This loop below should be run at its possible highest rate
    // Serial.println("Waiting ...");
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

    ////////////////////////////////////////////////////////////////////////
    // bool msg_found = false;
    // mavlink_message_t rx_msg;
    // mavlink_status_t rx_status;

    // Serial.println("Waiting ...");
    // while (!msg_found) {
    //   mavlink_message_t msg;
    //   mavlink_status_t status;
     
    //   while(Serial.available()) {
    //     uint8_t c = Serial.read();

    //     // Try to get a new message
    //     if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
    //       // // Handle message
    //       // switch(msg.msgid) {
    //       //   case MAVLINK_MSG_ID_HEARTBEAT:{
    //       //   // E.g. read GCS heartbeat and go into
    //       //                       // comm lost mode if timer times out
    //       //     break;
    //       //   }
    //       //   default: {
    //       //     break;
    //       //   }
    //       // }
    //       msg_found = true;
    //       break;
    //     }
    //     // msg_found = true;
    //     // break;
    //   }// while(Serial.available())  

    //   rx_msg = msg;
    //   rx_status = status;
    // }// while (!msg_found) 
    
    ////////////////////////////////////////////////////////////////////////
    bool msg_found = false;
    mavlink_message_t rx_msg;
    mavlink_status_t rx_status;

    Serial.println("Waiting ...");
    while (!msg_found) {
      if (Serial.available() > 0) {
        if (mavlink_parse_char(MAVLINK_COMM_0, Serial.read(), &rx_msg, &rx_status)) {
          if ( (rx_msg.msgid==MAVLINK_MSG_ID_ATTITUDE) ) {
            msg_found = true;
          }
        }
        // break;
      }
    }// while (!msg_found)

    //
    mavlink_attitude_t msg;
    mavlink_msg_attitude_decode(&rx_msg, &msg);

    // Serial.println("----------------------------------");
    // Serial.print("msg.roll= "); Serial.println(msg.roll);
    // Serial.print("msg.pitch= "); Serial.println(msg.pitch);
    // Serial.print("msg.yaw= "); Serial.println(msg.yaw);
    Serial.print("msg.rollspeed= "); Serial.println(msg.rollspeed);
    // Serial.print("msg.pitchspeed= "); Serial.println(msg.pitchspeed);
    // Serial.print("msg.yawspeed= "); Serial.println(msg.yawspeed);

    Serial.print("ctr= "); Serial.println(ctr);
    ++ctr;
    delay(100);
  }

  return 0;
}
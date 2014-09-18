#include "chr_um6.hpp"

using namespace crim;

CHR_UM6::CHR_UM6() {
  Serial2.begin(115200);
}

CHR_UM6::~CHR_UM6() {
  Serial2.end();
}

String CHR_UM6::firmware_version() {
  String fw_rev;
  CHR_UM6_packet packet;
  bool found = false;
  
  found = read(0x00, 0xAA, &packet);
  
  if (found) {
    for (uint8_t i=0; i<packet.data_len; ++i) {
      fw_rev += (char) packet.data[i];
    }
  }
  else {
    fw_rev = "Unknown";
  }
  
  return fw_rev;
}

EulerAngle CHR_UM6::euler() {
  //Serial.println("CHR_UM6::euler(): BEGIN");
  
  const double kScale = 0.01098632812;// = 360 deg/2^15, due to the value is stored in 16-bit 2's complement integer
  const double kOneDegreeInRadian = 0.0174532925;
  
  EulerAngle euler;
  CHR_UM6_packet packet;
  
  uint8_t type = 0b01001000;// is_batch= 1, number of registers= 2
  uint8_t addr = 0x62;
  
  bool found;
  found = read(type, addr, &packet);
  
  if (found) {
    // Roll angle  --------------------------------------------------
    uint16_t hi_byte_roll = (packet.data[0] << 8) | 0x00FF;
    uint16_t lo_byte_roll = packet.data[1] | 0xFF00;
    uint16_t u_roll = hi_byte_roll & lo_byte_roll;
    int16_t roll = u_roll;

    //cout << "hi_byte_roll= " << std::hex << std::showbase << hi_byte_roll << endl;
    //cout << "lo_byte_roll= " << std::hex << std::showbase << lo_byte_roll << endl;
    //cout << "u_roll= " << std::hex << std::showbase << u_roll << endl;
    //cout << "roll= " << std::dec << std::showbase << roll << endl;
    
    euler.roll = roll * kScale * kOneDegreeInRadian;
    
    // Pitch angle  -------------------------------------------------
    uint16_t hi_byte_pitch = (packet.data[2] << 8) | 0x00FF;
    uint16_t lo_byte_pitch = packet.data[3] | 0xFF00;
    uint16_t u_pitch = hi_byte_pitch & lo_byte_pitch;
    int16_t pitch = u_pitch;
    
    //cout << "hi_byte_pitch= " << std::hex << std::showbase << hi_byte_pitch << endl;
    //cout << "lo_byte_pitch= " << std::hex << std::showbase << lo_byte_pitch << endl;
    //cout << "u_pitch= " << std::hex << std::showbase << u_pitch << endl;
    //cout << "pitch= " << std::dec << std::showbase << pitch << endl;
    
    euler.pitch = pitch * kScale * kOneDegreeInRadian;
    
    // Yaw angle  ---------------------------------------------------
    uint16_t hi_byte_yaw = (packet.data[4] << 8) | 0x00FF;
    uint16_t lo_byte_yaw = packet.data[5] | 0xFF00;
    uint16_t u_yaw = hi_byte_yaw & lo_byte_yaw;
    int16_t yaw = u_yaw;
    
    //cout << "hi_byte_yaw= " << std::hex << std::showbase << hi_byte_yaw << endl;
    //cout << "lo_byte_yaw= " << std::hex << std::showbase << lo_byte_yaw << endl;
    //cout << "u_yaw= " << std::hex << std::showbase << u_yaw << endl;
    //cout << "yaw= " << std::dec << std::showbase << yaw << endl;
    
    euler.yaw = yaw * kScale * kOneDegreeInRadian;
  }
  
  //Serial.println("CHR_UM6::euler(): END");
  return euler;
}

 bool CHR_UM6::calib() {
  zero_gyros();
  set_accel_ref();
  set_mag_ref();
  reset_ekf();
  
  // TODO @tttor: the delay becomes unnecessary once we know that all commands finish by receiving an appropriate packet
  delay(3000);
  
  return true;
}

bool CHR_UM6::zero_gyros() {
  Serial.print("zero_gyros()...");
  
  bool found = false;
  found = read(0x00, 0xAC);
  
  // TODO @tttor: wait for COMMAND_COMPLETE packet and a packet indicating that the command finishes
  delay(1000);
  
  Serial.println("END");
  return found;
}

bool CHR_UM6::set_accel_ref() {
  Serial.print("set_accel_ref()...");
  
  bool found = false;
  found = read(0x00, 0xAF);
  
  // TODO @tttor: wait for COMMAND_COMPLETE packet and a packet indicating that the command finishes
  delay(1000);
  
  Serial.println("END");
  return found;
}

bool CHR_UM6::set_mag_ref() {
  Serial.print("set_mag_ref()...");
  
  bool found = false;
  found = read(0x00, 0xB0);
  
  // TODO @tttor: wait for COMMAND_COMPLETE packet and a packet indicating that the command finishes
  delay(1000);
  
  Serial.println("END");
  return found;
}

bool CHR_UM6::reset_ekf() {
  Serial.print("reset_ekf()...");
  
  bool found = false;
  found = read(0x00, 0xAD);
  
  // TODO @tttor: wait for COMMAND_COMPLETE packet and a packet indicating that the command finishes
  delay(1000);
  
  Serial.println("END");
  return found;
}

 bool CHR_UM6::read(uint8_t type, uint8_t addr, CHR_UM6_packet* packet) {
  //Serial.println("read(): BEGIN");
  
  // Block till the desired packet is found
  bool desired_packet_found = false;
  while ( !desired_packet_found ) {
    // Initiate the read mode
    CHR_UM6_packet tx_packet;
    tx_packet.type = type;
    tx_packet.addr = addr;
  
    tx_packet.send();
    
    // Handle the response, blocking till we get the desired packet
    CHR_UM6_packet rx_packet;
    bool packet_found = false;
    
    packet_found = rx_packet.wait();// block till one valid packet is received 
    
    if (packet_found) {
      //Serial.print("tx_packet.addr= "); Serial.println(tx_packet.addr, HEX);
      //Serial.print("rx_packet.addr= "); Serial.println(rx_packet.addr, HEX);
      
      if( rx_packet.addr == tx_packet.addr )
      {
        desired_packet_found = true;
        
        if (packet != 0)
          *packet = rx_packet;
      }
    }
  }
  
  //Serial.println("read(): END");
  return desired_packet_found;
}

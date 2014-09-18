#include "chr_um6_packet.hpp"

using namespace crim;
using namespace std;

const uint8_t CHR_UM6_packet::header_len = 3;
const uint8_t CHR_UM6_packet::type_len = 1;
const uint8_t CHR_UM6_packet::addr_len = 1;
const uint8_t CHR_UM6_packet::max_data_len = 64;
const uint8_t CHR_UM6_packet::checksum_len = 2;

CHR_UM6_packet::CHR_UM6_packet() {
  header[0] = 's';// 0x73
  header[1] = 'n';// 0x6E
  header[2] = 'p';// 0x70
  
  type = 0;
  addr = 0;
  data_len = 0;
  
  // Note that we do not explicitly initalize the variable "data"
}

CHR_UM6_packet::~CHR_UM6_packet() {
  //
}

bool CHR_UM6_packet::send() {
  // 
  uint16_t checksum = compute_checksum();
  uint8_t hi_byte_checksum = (checksum >> 8) & 0x00FF;
  uint8_t lo_byte_checksum = checksum & 0x00FF;
  
  // Wrap into a buffer
  const uint8_t buffer_len = CHR_UM6_packet::header_len + CHR_UM6_packet::type_len + CHR_UM6_packet::addr_len + data_len + CHR_UM6_packet::checksum_len;
  uint8_t buffer[buffer_len];
  
  for (uint8_t i=0; i<CHR_UM6_packet::header_len; ++i) buffer[i] = header[i];
  buffer[3] = type;
  buffer[4] = addr;
  for (uint8_t i=0; i<data_len; ++i) buffer[i+(CHR_UM6_packet::header_len + CHR_UM6_packet::type_len + CHR_UM6_packet::addr_len)] = data[i];
  buffer[CHR_UM6_packet::header_len + CHR_UM6_packet::type_len + CHR_UM6_packet::addr_len + data_len] = hi_byte_checksum;
  buffer[CHR_UM6_packet::header_len + CHR_UM6_packet::type_len + CHR_UM6_packet::addr_len + data_len + (CHR_UM6_packet::checksum_len)/2] = lo_byte_checksum;
  
  //
  Serial2.write(buffer, buffer_len);
}

bool CHR_UM6_packet::wait() {
  //Serial.println("wait(): BEGIN");
  
  // Block till a valid packet is found
  bool packet_found = false;
  while ( !packet_found ) {
    const uint8_t rx_buffer_len = 300;
    uint8_t rx_buffer[rx_buffer_len];
    
    // Fill in the buffer
    uint8_t rx_buffer_idx = 0;
    while (true) {// Note: This loop is critical; _must_ be as fast as possible: No delay, otherwise, some characters are likely missed
      if (Serial2.available() > 0) {
        uint8_t rx_datum = Serial2.read();
        //Serial.print("rx_datum= "); Serial.println(rx_datum, HEX);
        
        if (rx_buffer_idx < rx_buffer_len) {
          rx_buffer[rx_buffer_idx] = rx_datum;
          ++rx_buffer_idx;
        }
        else {// buffer is full
          break;
        }
      }
    }
      
    // Parse the buffer
    uint8_t parsing_status;    
    parsing_status = parse_serial_data(rx_buffer, rx_buffer_len);
    //Serial.print("parse_serial_data --> status= "); Serial.println(parsing_status, DEC);
    
    if (parsing_status == 0) {
      packet_found = true;
      //Serial.println("====================================A PACKET IS FOUND");
    }  
  }
  
  //Serial.println("wait(): END");
  return packet_found;
}

uint16_t CHR_UM6_packet::compute_checksum() {
  uint16_t checksum = 0;  
      
  for (uint8_t i=0; i<CHR_UM6_packet::header_len; ++i) checksum += header[i];
  checksum += type;
  checksum += addr;
  for (uint16_t i=0; i<data_len; ++i) checksum += data[i];
  
  return checksum;
}

uint8_t CHR_UM6_packet::parse_serial_data(uint8_t* rx_buffer, uint8_t rx_buffer_len) {
  uint8_t index;
  
  // Make sure that the data buffer provided is long enough to contain a full packet
  // The minimum packet length is 7 bytes, without data bytes: 's', 'n', 'p', packet type (PT), Address, Checksum 1, Checksum 0
  if( rx_buffer_len < 7 )
  {
    return 1;
  }
  
  // Try to find the ‘snp’ start sequence for the packet
  for( index = 0; index < (rx_buffer_len - 2); index++ )
  {
    // Check for ‘snp’. If found, immediately exit the loop
    if( rx_buffer[index] == 's' && rx_buffer[index+1] == 'n' && rx_buffer[index+2] == 'p' )
    {
      break;
    }
  }
  
  uint8_t packet_index = index;
  // Check to see if the variable ‘packet_index’ is equal to (rx_buffer_len - 2). 
  // If it is, then the above loop executed to completion and never found a packet header.
  if( packet_index == (rx_buffer_len - 2) )
  {
    return 2;
  }
  
  // If we get here, a packet header was found. Now check to see if we have enough room
  // left in the buffer to contain a full packet. Note that at this point, the variable ‘packet_index’
  // contains the location of the ‘s’ character in the buffer (the first byte in the header)
  if( (rx_buffer_len - packet_index) < 7 )
  {
    return 3;
  }
  
  // We’ve found a packet header, and there is enough space left in the buffer for at least the smallest allowable packet length (7 bytes). 
  // Pull out the packet type byte to determine the actual length of this packet
  uint8_t PT = rx_buffer[packet_index + 3];// i.e. the fourth byte in the packet
  
  // Do some bit-level manipulation to determine if the packet contains data and if it is a batch
  // We have to do this because the individual bits in the PT byte specify the contents of the
  // packet.
  uint8_t packet_has_data = (PT >> 7) & 0x01; // Check bit 7 (HAS_DATA)
  uint8_t packet_is_batch = (PT >> 6) & 0x01; // Check bit 6 (IS_BATCH)
  uint8_t batch_length = (PT >> 2) & 0x0F; // Extract the batch length (bits 2 through 5)

  // Now finally figure out the actual packet length
  if( packet_has_data )
  {
    if( packet_is_batch )
    {
      // Packet has data and is a batch. This means it contains ‘batch_length' registers, each
      // of which has a length of 4 bytes
      data_len = 4*batch_length;
    }
    else // Packet has data but is not a batch. This means it contains one register (4 bytes)
    {
      data_len = 4;
    }
  }
  else // Packet has no data
  {
    data_len = 0;
  }
  
  // At this point, we know exactly how long the packet is. 
  // Now we can check to make sure we have enough data for the full packet.
  if( (rx_buffer_len - packet_index) < (data_len + 5) )// plus 5 for bytes for: 's', 'n', 'p', packet type (PT), Address
  {
    return 3;
  }
  
  // If we get here, we know that we have a full packet in the buffer. 
  // All that remains is to pull out the data and make sure the checksum is good.
  // Start by extracting all the data
  type = PT;
  addr = rx_buffer[packet_index + 4];
  
  // Get the data bytes and compute the checksum all in one step
  uint16_t computed_checksum = 's' + 'n' + 'p' + type + addr;
  
  for( index = 0; index < data_len; index++ )
  {
    // Copy the data into the packet structure’s data array
    data[index] = rx_buffer[packet_index + 5 + index];
    // Add the new byte to the checksum
    computed_checksum += data[index];
  }

  // Now see if our computed checksum matches the received checksum
  // First extract the checksum from the packet
  uint16_t received_checksum = (rx_buffer[packet_index + 5 + data_len] << 8);
  received_checksum |= rx_buffer[packet_index + 6 + data_len];
  // Now check to see if they don’t match
  if( received_checksum != computed_checksum )
  {
    return 4;
  }
  
  // At this point, we’ve received a full packet with a good checksum. 
  // It is already fully parsed and copied to the ‘packet’ structure, so return 0 to indicate that a packet was processed.
  return 0;
}

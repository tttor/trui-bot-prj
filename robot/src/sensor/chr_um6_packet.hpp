// @author vektor dewanto
#ifndef CHR_UM6_PACKET
#define CHR_UM6_PACKET

#include <arduino-core/Arduino.h>
#include <stdint.h>

namespace crim {

/**
 * Units are all in radians
 */  
struct EulerAngle {
  double roll;
  double pitch;
  double yaw;
};

struct Quaternion {
  double x;
  double y;
  double z;
  double w;
};

/**
 * Structure for holding received packet information
 * three types of registers onboard the UM6: configuration registers, data registers, and command registers.
 * 
 * When communication is performed over the UART, data transmitted and received by the UM6 is formatted into packets containing:
 * 1. The three character start sequence 's', 'n', 'p' to indicate the start of a new packet (i.e. start new packet)
 * 2. A "packet type" (PT) byte describing the function and length of the packet
 * 3. An address byte indicating the address of the register or command
 * 4. A sequence of data bytes, the length of which is specified in the PT byte, maximum of 2^4=16 registers (in the batch mode); that is 16 * 4 bytes = 64 bytes
 * 5. A two-byte checksum for error-detection
 * 
 * The current implementation (Feb 14, 2014) required that the communication happens via Serial2 with the baudrate of 115200
 */
class CHR_UM6_packet {
 public:
  uint8_t header[3];// const header_len = 3
  uint8_t type;
  uint8_t addr;
  uint8_t data[64];// const max_data_len = 64
  uint8_t data_len;
  
  // All lengths below are in bytes
  static const uint8_t header_len;
  static const uint8_t type_len;
  static const uint8_t addr_len;
  static const uint8_t max_data_len;
  static const uint8_t checksum_len;
  
  CHR_UM6_packet(); 
  ~CHR_UM6_packet();
  
  /**
   * Require the comm is using Serial2
   */
  bool send();
  
  /**
   * @brief block till a valid (not necessarily with the desired addr) packet is found
   * Require the comm is using Serial2 with the baudrate of 115200
   */
  bool wait();
  
 private:
  /**
   * A simple checksum: addition of containing bytes (exclusing the checksum bytes) in a packet
   * 
   */
  uint16_t compute_checksum();
  
  /**
  * @brief parse_serial_data (adopted from the UM6 Datasheet)
  * This function parses the data in ‘rx_data’ with length ‘rx_length’ and attempts to find a packet
  * in the data. If a packet is found, the structure ‘packet’ is filled with the packet data.
  * If there is not enough data for a full packet in the provided array, parse_serial_data returns 1.
  * If there is enough data, but no packet header was found, parse_serial_data returns 2.
  * If a packet header was found, but there was insufficient data to parse the whole packet,
  * then parse_serial_data returns 3. This could happen if not all of the serial data has been
  * received when parse_serial_data is called.
  * If a packet was received, but the checksum was bad, parse_serial_data returns 4.
  * If a good packet was received, parse_serial_data fills the UM6_packet structure and returns 0.
  * 
  * Regardless of whether the UART or the SPI bus is used, data is retrieved from the UM6 by reading from onboard 32-bit registers.
  * The UM6 UART operates at a 3.3V logic level with 8 data bits, 1 stop bit, and no parity.
  * By default, the serial baud rate is set at 115200 baud, but the baud rate can be changed by the end user if desired by writing to the UM6_COMMUNICATION register.
  * 
  * UART communication can operate in one of two modes: Broadcast and Listen mode.
  * In Broadcast Mode, the UM6 will automatically transmit sensor data at a user-configurable rate ranging from 20 Hz to 300 Hz. 
  * The broadcast rate can be adjusted by writing to the UM6_COMMUNICATION register.
  * 
  * In Broadcast Mode, any combination of raw or processed sensor data, orientation estimates, and estimator covariance can be transmitted automatically. 
  * The data channels that are enabled are called "active channels." 
  * By default, only processed rate gyro data, accelerometer data, magnetometer data, and estimated angles are transmitted active. 
  * This can be changed by writing to the UM6_COMMUNICATION register. 
  * (Note: "Processed" data is sensor data to which calibration has been applied. "Raw" data is the data measured directly by the sensor). 
  * Each batch of data is transmitted in its own packet - thus, rate gyro data occupies one packet, accelerometer data occupies another, etc.)
  * 
  * In contrast to Broadcast Mode, Listen Mode causes the UM6 to wait for requests over the UART before transmitting data. 
  * A GET_DATA command causes all active channel data to be transmitted (the same data that would be transmitted if Broadcast Mode were enabled). 
  * Alternatively, specific data registers can be queried while the device is in Listen Mode. 
  * The UM6 is in Broadcast Mode by default. This setting can be changed by writing to the UM6_COMMUNICATION register.
  */
  uint8_t parse_serial_data(uint8_t* rx_data, uint8_t rx_length);
};

}// namespace crim

#endif

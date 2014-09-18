// @author vektor dewanto
#ifndef CHR_UM6_H
#define CHR_UM6_H

#include "chr_um6_packet.hpp"

namespace crim{

class CHR_UM6 {
 public:
  CHR_UM6();
  ~CHR_UM6();
  
  String firmware_version();
  EulerAngle euler();
  
  /**
   * @brief do four steps:
   * 1. zero_gyros
   * 2. set_accel_ref
   * 3. set_mag_ref
   * 4. reset_ekf
   * 
   * Command registers are technically not registers at all, 
   * but they provide a convenient way to trigger specific commands onboard the UM6 using the same communication interface used to write to and read from registers. 
   * Commands are initiated by executing a read command for the relevant command address using the UART.
   */
  bool calib();
  
 private:
  /**
  * @brief Causes the UM6 to start executing the zero gyros command. 
  * Once the  operation begins, a COMMAND_COMPLETE packet is sent. 
  * After the  command finishes, the gyro bias registers are transmitted over the UART.
  */ 
  bool zero_gyros();
  
  /**
   * 
   */
  bool set_accel_ref();
  
  /**
   * 
   */
  bool set_mag_ref();
  
  /**
   * 
   */
  bool reset_ekf();
  
  /**
   * @brief read one or more registers
   * Block till the desired packet containing the desired register is found
   */
  bool read(uint8_t type, uint8_t addr, CHR_UM6_packet* packet=0);
};

}// namespace crim

#endif

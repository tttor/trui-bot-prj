// @author vektor dewanto
#ifndef ARDUINO_MAVLINK_PACKET_HANDLER
#define ARDUINO_MAVLINK_PACKET_HANDLER

#include <arduino-core/Arduino.h>
#include <mavlink/v1.0/common/mavlink.h>

namespace crim {

class ArduinoMavlinkPacketHandler {
 public:
  /**
   * @brief The mavlink_system here is used when sending packets
   */
  ArduinoMavlinkPacketHandler(mavlink_system_t mavlink_system, String port, uint32_t baud_rate=9600);
  
  /**
   * @brief
   */
  ~ArduinoMavlinkPacketHandler();

  /**
   * @brief
   */
  void send();
    
  /**
   * @brief Euler angle attitude
   */
  void wrap(mavlink_attitude_t raw_msg);

  /**
   * @brief Quaternion attitude
   */
  void wrap(mavlink_attitude_quaternion_t raw_msg);
  
  /**
   * @brief 
   */
  void wrap(mavlink_global_position_int_t raw_msg);

  /**
   * @brief 
   */
  void wrap(mavlink_mission_set_current_t raw_msg);
  
  /**
   * @brief 
   */
  void wrap(mavlink_rc_channels_raw_t raw_msg);
      
  /**
   * @brief For Euler angle attitude. This blocks till the desired msg has been received
   */
  void wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_attitude_t* the_msg);
  
  /**
   * @brief For Quaternion attitude. This blocks till the desired msg has been received
   */
  void wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_attitude_quaternion_t* the_msg);
  
  /**
   * @brief 
   */
  void wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_global_position_int_t* the_msg);
  
  /**
   * @brief 
   */
  void wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_mission_set_current_t* the_msg);
  
  /**
   * @brief 
   */
  void wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_rc_channels_raw_t* the_msg);
 
 private:
  mavlink_system_t mavlink_system_;
  String port_;
  uint32_t baud_rate_;
  mavlink_message_t* msg_;
  
  /**
   * @brief This blocks till the desired msg has been received
   */
  void wait(uint8_t sysid, uint8_t compid, uint8_t msgid, mavlink_message_t* msg);
};

}// namespace crim

#endif

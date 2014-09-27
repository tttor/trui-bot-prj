/*
 * Author: Vektor Dewanto, adopted from PR2's by Sachin Chitta and Matthew Piccoli
 */
#ifndef RBMT_BASE_KINEMATICS_H
#define RBMT_BASE_KINEMATICS_H

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

namespace rbmt_base_ctrl {
class Wheel;
class BaseKinematics;

/*! \class
 \brief This class keeps track of the wheels
 */
class Wheel {
 public:
  Wheel();
  ~Wheel();
  
  /*!
  * \brief Loads wheel's information from the xml description file and param server
  */
  bool init();

  /*!
   * \brief name of the joint
   */
  std::string joint_name;

  /*!
   * \brief name of the link
   */
  std::string link_name;

    /*!
   * \brief wheel radius scale (based on the default wheel radius in Basekinematics)
   */
  double wheel_radius;

  /*!
   * \brief actual wheel speeds
   */
  double wheel_speed_actual;

  /*!
   * \brief desired wheel speed
   */
  double wheel_speed_cmd;

  /*!
   * \brief difference between desired and actual speed
   */
  double wheel_speed_error;

  /*!
   * \brief specifies the default direction of the wheel
   */
  int direction_multiplier;
 private:
};

/*! \class
 \brief This class includes common functions used by the base controller and odometry
 */
class BaseKinematics {
 public:
  BaseKinematics();
  ~BaseKinematics();
  
  /*!
   * \brief Loads BaseKinematic's information, 
   * now hardcoded manually, can come from the xml description file and param server
   * @return Successful init
   */
  bool init();

  /*!
  * \brief vector of every wheel attached to the base
  */
  std::vector<Wheel> wheels;

  /*!
  * \brief Name(string id) of the robot base frame
  */
    std::string frame_id;
 private:
};

} // namespace
#endif

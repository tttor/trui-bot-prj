#include <rbmt_teleop/rbmt_teleop_translator.h>

namespace rbmt_teleop {

TeleopTranslator::TeleopTranslator(ros::NodeHandle nh): nh_(nh) {
  //
  n_axes_ = 8;
  n_button_ = 11;

  axes_.resize(n_axes_);
  axis_mins_.resize(n_axes_);
  axis_maxs_.resize(n_axes_);
  axis_normals_.resize(n_axes_);
  buttons_.resize(n_button_);

  //
  axis_mins_.at(0) = -1.0;
  axis_maxs_.at(0) =  1.0;
  axis_normals_.at(0) = 0.0;

  axis_mins_.at(1) = -1.0;
  axis_maxs_.at(1) =  1.0;
  axis_normals_.at(1) = 0.0;

  axis_mins_.at(2) = -1.0;
  axis_maxs_.at(2) =  1.0;
  axis_normals_.at(2) = 1.0;

  axis_mins_.at(5) = -1.0;
  axis_maxs_.at(5) =  1.0;
  axis_normals_.at(5) = 1.0;

  vel_param_["x_vel_max"] =  1.0;
  vel_param_["x_vel_min"] = -1.0 * vel_param_["x_vel_max"];
  vel_param_["y_vel_max"] =  vel_param_["x_vel_max"];
  vel_param_["y_vel_min"] = -1.0 * vel_param_["y_vel_max"];
  vel_param_["theta_vel_min"] = 0.0;
  vel_param_["theta_vel_max"] = 0.25 * M_PI;

  //
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTranslator::joy_sub_cb, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
}

TeleopTranslator::~TeleopTranslator() {

}

void TeleopTranslator::joy_sub_cb(const sensor_msgs::JoyConstPtr& msg) {
  axes_ = msg->axes;
  buttons_ = msg->buttons;
}

void TeleopTranslator::run() {
  const bool debug = false;

  // axes(5): RT _must_ be initialized because _before_ first update _only_, axes_.at(5) has a normal value of 0, plus, if another axes is pushed before RT is initialized, then RT changes to not-normal value; weird!
  bool RT_initialized = false;
  bool LT_initialized = false;

  ROS_INFO("Waiting for LT (axis(2)) and RT (axis(5)) to be initialized.");
  while (ros::ok() and !RT_initialized and !LT_initialized) {
    if (axes_.at(5)==axis_normals_.at(5) && axes_.at(2)==axis_normals_.at(2)) break;
    ros::spinOnce();
  }
  ROS_INFO("LT (axis(2)) and RT (axis(5)) is initialized.");

  ros::Rate rate(10);
  while (ros::ok()) {
    // Set the values based on joy readings
    double x_vel, y_vel, theta_vel;
    
    x_vel = (reverse(axes_.at(0)) / (axis_range_ratio(0)*axis_range(0))) * (axis_range_ratio(0)*vel_range("x_vel"));// reversed due to reversed reading
    y_vel = (axes_.at(1) / (axis_range_ratio(1)*axis_range(1))) * (axis_range_ratio(1)*vel_range("y_vel"));
    theta_vel = reverse( (std::abs(axes_.at(5)-1.0) / (axis_range_ratio(5)*axis_range(5))) * (axis_range_ratio(5)*vel_range("theta_vel")) )
     + (std::abs(axes_.at(2)-1.0) / (axis_range_ratio(2)*axis_range(2))) * (axis_range_ratio(2)*vel_range("theta_vel"));// reversed due to positive RT should make CCW rotation
    
    // Publish
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = x_vel;
    cmd_vel.linear.y = y_vel;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = theta_vel;

    ROS_DEBUG_STREAM_COND(debug, "cmd_vel.linear.x= "<< cmd_vel.linear.x);
    ROS_DEBUG_STREAM_COND(debug, "cmd_vel.linear.y= "<< cmd_vel.linear.y);
    ROS_DEBUG_STREAM_COND(debug, "cmd_vel.angular.z= "<< cmd_vel.angular.z);

    cmd_vel_pub_.publish(cmd_vel);  

    if(buttons_.at(3)==1) {
      const std::string action_server_name = "move_base";
      MoveBaseClient ac(action_server_name);
      ac.waitForServer(); 
      move_base_msgs::MoveBaseGoal goal;

      goal.target_pose.header.frame_id = "map";// the reference frame of the goal pose
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = 0.0;
      goal.target_pose.pose.position.y = 0.0;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      
      send_goal(goal, ac); 
    }

    //
    ros::spinOnce();
    rate.sleep();
  }
}

float TeleopTranslator::axis_range(const size_t& ith) {
  return axis_maxs_.at(ith) - axis_mins_.at(ith);
}

float TeleopTranslator::vel_range(const std::string type) {
  return vel_param_[std::string(type+"_max")] - vel_param_[std::string(type+"_min")]; 
}

float TeleopTranslator::reverse(const float& val) {
  return -1.0 * val;
}

float TeleopTranslator::axis_range_ratio(const size_t& ith) {
  return std::abs(axis_normals_.at(ith)-axis_mins_.at(ith)) / axis_range(ith);
}

size_t TeleopTranslator::send_goal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac) {
  ROS_INFO("send_goal(): BEGIN");
  size_t status = 0;
  
  //
  ac.sendGoal(goal);

  bool finished_before_timeout;
  finished_before_timeout = ac.waitForResult(ros::Duration(5.0));//TODO @tttor: may use a callback fo this

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_DEBUG("Action's terminal state= %s", state.toString().c_str());
      status = 1;
    }
    else {
      ROS_DEBUG("The base failed to move forward 2 meters for some reason");
      status = 2;
    }
  }
  else {
    ROS_WARN("TIMEOUT: time is up :(");
    status = 3;
  }

  ROS_INFO("send_goal(): BEGIN");
  return status;
}

}// namespace rbmt_teleop

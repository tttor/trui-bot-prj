#include <rbmt_service/rbmt_service.h>

namespace rbmt_service {

ServiceTeleop::ServiceTeleop(ros::NodeHandle nh): nh_(nh) {

  joy_sub_ = nh_.subscribe<std_msgs::Int16MultiArray>("read_joy",1, &ServiceTeleop::joy_sub_cb, this);//get controller command from /read_joy topic
  // kinect_sub_ = nh_.subscribe<geometry_msgs::Twist>("kinect_velocity",1, &ServiceTeleop::kinect_sub_cb, this);//get controller command from /read_joy topic
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("read_velocity", 100);
  // cmd_service_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("cmd_service", 100);
}

ServiceTeleop::~ServiceTeleop() {

}

// void ServiceTeleop::kinect_sub_cb(const geometry_msgs::Twist::ConstPtr& vel_msg) {
//   vel_kinect_x_ = vel_msg->linear.x;
//   vel_kinect_y_ = vel_msg->linear.y;
//   vel_kinect_z_ = vel_msg->angular.z;
// }

void ServiceTeleop::joy_sub_cb(const std_msgs::Int16MultiArray::ConstPtr& msg) {
  axisX_          = msg->data[0];
  axisY_          = msg->data[1];
  //= msg->data[2];
  //= msg->data[3];
  buttonTriangle_ = msg->data[4];
  buttonCross_    = msg->data[5];
  buttonSquare_   = msg->data[6];
  buttonCircle_   = msg->data[7];
  buttonDown_     = msg->data[8];
  buttonLeft_     = msg->data[9];
  buttonUp_       = msg->data[10];
  buttonRight_    = msg->data[11];
  buttonL1_       = msg->data[12];
  buttonL2_       = msg->data[13];
  buttonR1_       = msg->data[14];
  buttonR2_       = msg->data[15];
  buttonStart_    = msg->data[16];
  buttonSelect_   = msg->data[17];
  buttonL3_       = msg->data[18];
  buttonR3_       = msg->data[19];
  
  // From Arduino Program
    // joy.data[0]  = ps2x.Analog(PSS_LX);
    // joy.data[1]  = ps2x.Analog(PSS_LY);
    // joy.data[2]  = ps2x.Analog(PSS_RX);
    // joy.data[3]  = ps2x.Analog(PSS_RY);
    // joy.data[4]  = ps2x.Button(PSB_TRIANGLE);
    // joy.data[5]  = ps2x.Button(PSB_CROSS);
    // joy.data[6]  = ps2x.Button(PSB_SQUARE);
    // joy.data[7]  = ps2x.Button(PSB_CIRCLE);
    // joy.data[8]  = ps2x.Button(PSB_PAD_DOWN);
    // joy.data[9]  = ps2x.Button(PSB_PAD_LEFT);
    // joy.data[10] = ps2x.Button(PSB_PAD_UP);
    // joy.data[11] = ps2x.Button(PSB_PAD_RIGHT);
    // joy.data[12] = ps2x.Button(PSB_L1);
    // joy.data[13] = ps2x.Button(PSB_L2);
    // joy.data[14] = ps2x.Button(PSB_R1);
    // joy.data[15] = ps2x.Button(PSB_R2);
    // joy.data[16] = ps2x.Button(PSB_START);
    // joy.data[17] = ps2x.Button(PSB_SELECT);
    // joy.data[18] = ps2x.Button(PSB_L3);
    // joy.data[19] = ps2x.Button(PSB_R3);
}

void ServiceTeleop::run(ros::Rate rate) {
  const bool debug = false;
  float serviceByte_ = 0;

  // axes(5): RT _must_ be initialized because _before_ first update _only_, axes_.at(5) has a normal value of 0, plus, if another axes is pushed before RT is initialized, then RT changes to not-normal value; weird!
  bool RT_initialized = false;
  bool LT_initialized = false;
  bool squareCount, triangleCount,circleCount, crossCount, R1Count;

  ROS_INFO("This is RBMT02, Lock and Load");
  // ROS_INFO("Waiting for LT (axis(2)) and RT (axis(5)) to be initialized.");
  // while (ros::ok() and !RT_initialized and !LT_initialized) {
  //   if (axes_.at(5)==axis_normals_.at(5) && axes_.at(2)==axis_normals_.at(2)) break;
  //   ros::spinOnce();
  // }
  // ROS_INFO("LT (axis(2)) and RT (axis(5)) is initialized.");
  serviceByte_ = 0;
  while (ros::ok()) {
    // Set the values based on joy readings
    float speedX_,speedY_,speedW_;
        
    // Publish
    geometry_msgs::Twist cmd_vel;

    if(axisX_ <= 148 and axisX_ >= 108) axisX_ = 128;
    if(axisY_ <= 148 and axisY_ >= 108) axisY_ = 128;
    speedX_ = float(axisX_-128)/128 * 1.5;///128 * 1.5);//4.5;//map(presentPosition_VX, -128, 127, -4.5, 4.5);
    speedY_ = - float(axisY_-128)/128 * 1.5;///128 * 1.5);//map(presentPosition_VY, -128, 128, -5, 5);
    if(buttonR2_ == 1) speedW_ = -1.5; else if(buttonL2_ == 1) speedW_ = 1.5; else speedW_ = 0;

    if(buttonL1_ == 1){
      speedX_ = 0.7*speedX_;
      speedY_ = 0.7*speedY_;
    }

    if(buttonTriangle_ == 1){

      cmd_vel.linear.x = vel_kinect_x_; //x_vel;
      cmd_vel.linear.y = vel_kinect_y_;
      cmd_vel.linear.z = 0;
      cmd_vel.angular.x = 0;
      cmd_vel.angular.y = 0;
      cmd_vel.angular.z = vel_kinect_z_;//theta_vel;
    
    }

    else{

      if(buttonSquare_ == 1 && squareCount == 0){
        squareCount = 1;
        serviceByte_ = serviceByte_ + 1;
        ROS_INFO("buttonSquare_ is a Go");
        }
      else if(buttonSquare_ == 0 && squareCount == 1){
        squareCount = 0;
        serviceByte_ = serviceByte_ - 1;
      }

      if(buttonCross_ == 1 && crossCount == 0){
        crossCount = 1;
        serviceByte_ = serviceByte_ + 2;
        ROS_INFO("buttonCross_ is a Go");
        }
      else if(buttonCross_ == 0 && crossCount == 1){
        crossCount = 0;
        serviceByte_ = serviceByte_ -2;
      }

      if(buttonCircle_ == 1 && circleCount == 0){
        circleCount = 1;
        serviceByte_ = serviceByte_ +4;
        ROS_INFO("buttonCircle_ is a Go");
        }
      else if(buttonCircle_ == 0 && circleCount == 1){
        circleCount = 0;
        serviceByte_ = serviceByte_ -4;
      }

      if(buttonR1_ == 1 && R1Count == 0){
        R1Count = 1;
        serviceByte_ = serviceByte_ +8;
        ROS_INFO("buttonCircle_ is a Go");
        }
      else if(buttonR1_ == 0 && R1Count == 1){
        R1Count = 0;
        serviceByte_ = serviceByte_ -8;
      }

      cmd_vel.linear.x = speedX_; //x_vel;
      cmd_vel.linear.y = speedY_;
      cmd_vel.linear.z = 0;
      cmd_vel.angular.x = buttonR1_;
      cmd_vel.angular.y = serviceByte_;//no offset after i added the R1 button hmmmm..// + 2; //Seems to have a consistent -2 offset. Don't know why.
      cmd_vel.angular.z = speedW_;//theta_vel;
      // ROS_INFO("%f",cmd_vel.angular.y);
    }


    
    //ROS_DEBUG_STREAM_COND(debug, "cmd_vel.linear.x= "<< cmd_vel.linear.x);
    //ROS_DEBUG_STREAM_COND(debug, "cmd_vel.linear.y= "<< cmd_vel.linear.y);
    //ROS_DEBUG_STREAM_COND(debug, "cmd_vel.angular.z= "<< cmd_vel.angular.z);

    cmd_vel_pub_.publish(cmd_vel);  

    // if(buttons_.at(num_["button_LB"])==1) {
    //   const std::string action_server_name = "move_base";
    //   MoveBaseClient ac(action_server_name);
    //   ac.waitForServer(); 
    //   move_base_msgs::MoveBaseGoal goal;

    //   goal.target_pose.header.frame_id = "map";// the reference frame of the goal pose
    //   goal.target_pose.header.stamp = ros::Time::now();
    //   goal.target_pose.pose.position.x = 0.0;
    //   goal.target_pose.pose.position.y = 0.0;
    //   goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      
    //   send_goal(goal, ac); 
    // }

    // if (buttons_.at(num_["button_RB"])==1) {
    //   // TODO @tttor: Replace with a compact (customized) msg; 
    //   // The choice for geometry_msgs::PoseStamped is because, for now, we use mavros built-in plugin: setpoint_position
    //   geometry_msgs::PoseStamped spose;

    //   //
    //   const double yaw = M_PI;// this mean do-service
    //   tf::Quaternion q = tf::createQuaternionFromYaw(yaw);

    //   spose.pose.orientation.x = q.x();
    //   spose.pose.orientation.y = q.y();
    //   spose.pose.orientation.z = q.z();
    //   spose.pose.orientation.w = q.w();

    //   cmd_service_pub_.publish(spose);
    // }

    //
    ros::spinOnce();
    rate.sleep();
  }
}

// float ServiceTeleop::axis_range(const size_t& ith) {
//   return axis_maxs_.at(ith) - axis_mins_.at(ith);
// }

// float ServiceTeleop::vel_range(const std::string type) {
//   return vel_param_[std::string(type+"_max")] - vel_param_[std::string(type+"_min")]; 
// }

// float ServiceTeleop::reverse(const float& val) {
//   return -1.0 * val;
// }

// float ServiceTeleop::axis_range_ratio(const size_t& ith) {
//   return std::abs(axis_normals_.at(ith)-axis_mins_.at(ith)) / axis_range(ith);
// }

// size_t ServiceTeleop::send_goal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac) {
//   ROS_INFO("send_goal(): BEGIN");
//   size_t status = 0;
  
//   //
//   ac.sendGoal(goal);

//   bool finished_before_timeout;
//   finished_before_timeout = ac.waitForResult(ros::Duration(5.0));//TODO @tttor: may use a callback fo this

//   if (finished_before_timeout) {
//     actionlib::SimpleClientGoalState state = ac.getState();

//     if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
//       ROS_DEBUG("Action's terminal state= %s", state.toString().c_str());
//       status = 1;
//     }
//     else {
//       ROS_DEBUG("The base failed to move forward 2 meters for some reason");
//       status = 2;
//     }
//   }
//   else {
//     ROS_WARN("TIMEOUT: time is up :(");
//     status = 3;
//   }

//   ROS_INFO("send_goal(): BEGIN");
//   return status;
// }

}// namespace rbmt_teleop

#include <rbmt_base_ctrl/rbmt_base_ctrl.h>
#include <log4cxx/logger.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "base_ctrl");
  ros::NodeHandle nh;

  // log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  // my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  rbmt_base_ctrl::BaseCtrl base_ctrl(nh);
  ros::spin();

  return(0);
}
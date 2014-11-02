#include <rbmt_nav/navigator.h>
#include <log4cxx/logger.h>

int main(int argc, char** argv) {
  using namespace std;

  ros::init(argc, argv, "navigator");
  ros::NodeHandle nh;	

  // log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  // my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  rbmt_nav::Navigator nav(nh);
  ros::spin();

  return(0);
}
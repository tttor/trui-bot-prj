#include <rbmt_service/rbmt_service.h>
#include <log4cxx/logger.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "service_teleop");
  ros::NodeHandle nh;
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  rbmt_service::ServiceTeleop service_teleop(nh);
  service_teleop.run(ros::Rate(100));
  
  return(0);
}
#include <rbmt_teleop/rbmt_teleop.h>
#include <log4cxx/logger.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "teleop_trans");
  ros::NodeHandle nh;
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  rbmt_teleop::TeleopTranslator teleop_trans(nh);
  teleop_trans.run();
  
  return(0);
}
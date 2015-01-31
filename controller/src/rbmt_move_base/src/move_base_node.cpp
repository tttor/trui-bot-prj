#include <rbmt_move_base/move_base.h>
#include <log4cxx/logger.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "move_base");
  ros::NodeHandle nh;
  tf::TransformListener tf_listener(ros::Duration(10));

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  rbmt_move_base::MoveBase move_base(nh, tf_listener);
  ros::spin();

  return(0);
}
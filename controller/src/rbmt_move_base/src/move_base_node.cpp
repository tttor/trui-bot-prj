#include <rbmt_move_base/move_base.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "move_base");
  tf::TransformListener tf_listener(ros::Duration(10));

  rbmt_move_base::MoveBase move_base(tf_listener);
  ros::spin();

  return(0);
}
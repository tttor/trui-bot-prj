#include <rbmt_move_base/move_base.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "move_base");

  rbmt_move_base::MoveBase move_base;
  ros::spin();

  return(0);
}
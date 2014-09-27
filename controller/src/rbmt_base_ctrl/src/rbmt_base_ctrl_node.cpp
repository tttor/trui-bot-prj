#include <rbmt_base_ctrl/rbmt_base_ctrl.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "base_ctrl");
  ros::NodeHandle nh;

  rbmt_base_ctrl::BaseCtrl base_ctrl(nh);
  ros::spin();

  return(0);
}
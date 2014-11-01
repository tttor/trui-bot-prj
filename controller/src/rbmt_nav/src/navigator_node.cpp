#include <rbmt_nav/navigator.h>

int main(int argc, char** argv) {
  using namespace std;

  ros::init(argc, argv, "navigator");
  ros::NodeHandle nh;	

  rbmt_nav::Navigator nav(nh);

  return(0);

}
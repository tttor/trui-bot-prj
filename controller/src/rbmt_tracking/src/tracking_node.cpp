#include <rbmt_tracking/tracker.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "tracking");
  ros::NodeHandle nh;

  rbmt_tracking::Tracker tracker(nh);
  tracker.run(ros::Rate(10));

  return(0);
}
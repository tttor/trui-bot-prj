#include <rbmt_tracking/tracker.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "tracking");
  
  rbmt_tracking::Tracker tracker;
  ros::spin();

  return(0);
}
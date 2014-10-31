#include <rbmt_tracking/tracker.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "tracking");
  ros::NodeHandle nh;

  rbmt_tracking::Tracker tracker(nh);
  tracker.run();

  return(0);
}
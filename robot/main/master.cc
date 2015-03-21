/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
#include <arduino/Arduino.h>
#include <ros_lib/ros.h>
#include <ros_lib/std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("arduino", &str_msg);

char hello[20] = "Hello from arduino!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(100);
}

int main() {


  init();

  setup();
  while (true) {
    
  /* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */


loop();




  
  }
  return 0;
}
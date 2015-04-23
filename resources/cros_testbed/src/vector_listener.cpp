#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cros_testbed/DoubleVector.h"


void chatterCallback(const cros_testbed::DoubleVector::ConstPtr& msg)
{
  for(int i = 0; i < msg->val.size(); i++)
    ROS_INFO("I heard: [%f]", msg->val[i]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vector_listener");
  ros::NodeHandle n; 
  ros::Subscriber sub = n.subscribe("double_vector", 1000, chatterCallback);
  ros::spin();

  return 0;
}
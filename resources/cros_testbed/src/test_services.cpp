#include "ros/ros.h"
#include "cros_testbed/add_two_ints.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_services_list");

  /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }*/

  ros::NodeHandle n;
  ros::ServiceClient client;

  //Test add_two_ints service
  client = n.serviceClient<cros_testbed::add_two_ints>("add_two_ints");

  cros_testbed::add_two_ints srv_add_two_ints;
  srv_add_two_ints.request.a = 3;
  srv_add_two_ints.request.b = 5;

  if (client.call(srv_add_two_ints))
  {
    ROS_INFO("Sum: %ld", (long int)srv_add_two_ints.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}

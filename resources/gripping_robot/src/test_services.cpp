#include <ros/ros.h>
#include <gripping_robot/CloseGripper.h>
#include <gripping_robot/MoveArm.h>
#include <gripping_robot/OpenGripper.h>
#include <gripping_robot/Reconfigure.h>
#include <gripping_robot/RestPosition.h>
#include <gripping_robot/Transfer.h>
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

  //Test reconfigure service

  client = n.serviceClient<gripping_robot::Reconfigure>("reconfigure");

  gripping_robot::Reconfigure srv_reconfigure;
  srv_reconfigure.request.id = 0x123456789abcdef0;

  if (client.call(srv_reconfigure))
  {
    ROS_INFO("reconfigure(0x%016ld): reconfigured: %d",
            (int64_t)srv_reconfigure.request.id,
            (uint8_t)srv_reconfigure.response.reconfigured);
  }
  else
  {
    ROS_ERROR("Failed to call service reconfigure");
    return 1;
  }

  //Test close_clamps service

  client = n.serviceClient<gripping_robot::CloseGripper>("close_gripper");

  gripping_robot::CloseGripper srv_close_gripper;

  if (client.call(srv_close_gripper))
  {
    ROS_INFO("Response: %d", (uint8_t)srv_close_gripper.response.closed);
  }
  else
  {
    ROS_ERROR("Failed to call service close_gripper");
    return 1;
  }

  //Test open_clamps service

  client = n.serviceClient<gripping_robot::OpenGripper>("open_gripper");

  gripping_robot::OpenGripper srv_open_gripper;

  if (client.call(srv_open_gripper))
  {
    ROS_INFO("Opened: %d", (uint8_t)srv_open_gripper.response.opened);
  }
  else
  {
    ROS_ERROR("Failed to call service open_gripper");
    return 1;
  }

  //Test park service

  client = n.serviceClient<gripping_robot::RestPosition>("rest_position");

  gripping_robot::RestPosition srv_rest;

  if (client.call(srv_rest))
  {
    ROS_INFO("Parked: %d", (uint8_t)srv_rest.response.parked);
  }
  else
  {
    ROS_ERROR("Failed to call rest_position");
    return 1;
  }

  //Test transfer service

  client = n.serviceClient<gripping_robot::Transfer>("transfer");

  gripping_robot::Transfer srv_transfer;

  if (client.call(srv_transfer))
  {
    ROS_INFO("Transfered: %d", (uint8_t)srv_transfer.response.transfered);
  }
  else
  {
    ROS_ERROR("Failed to call service transfer");
    return 1;
  }

  //Test move_arm service

  client = n.serviceClient<gripping_robot::MoveArm>("move_arm");

  gripping_robot::MoveArm srv_move_arm;
  srv_move_arm.request.arm = -1;// metto apposta -1 perche' voglio solo fare test
                                // e non muovere veramente l'arm!
  srv_move_arm.request.rad = 2;
  srv_move_arm.request.velocity = 3;

  if (client.call(srv_move_arm))
  {
    ROS_INFO("move_arm(%d, %d, %d): positioned: %d",
            (int)srv_move_arm.request.arm,
            (int)srv_move_arm.request.rad,
            (int)srv_move_arm.request.velocity,
            (uint8_t)srv_move_arm.response.positioned);
  }
  else
  {
    ROS_ERROR("Failed to call service move_arm");
    return 1;
  }

  return 0;
}

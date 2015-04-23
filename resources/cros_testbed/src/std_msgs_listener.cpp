#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Char.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

void chatter1(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data ? "true" : "false");
}

void chatter2(const std_msgs::Byte::ConstPtr& msg)
{
  ROS_INFO("I heard: [%i]", msg->data);
}

void chatter3(const std_msgs::Char::ConstPtr& msg)
{
  ROS_INFO("I heard: [%i]", msg->data);
}

void chatter4(const std_msgs::Duration::ConstPtr& msg)
{
  ROS_INFO("I heard: sec=[%i] nsec=[%i]", msg->data.sec, msg->data.nsec);
}


void chatter5(const std_msgs::Header::ConstPtr& msg)
{
  ROS_INFO("I heard: seq = [%i], time = [%i, %i], frame_id = [%s]", msg->seq, msg->stamp.sec, msg->stamp.nsec, msg->frame_id.c_str());
}

void chatter6(const std_msgs::Int16::ConstPtr& msg)
{
  ROS_INFO("I heard: [%i]", msg->data);
}

void chatter7(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("I heard: [%i]", msg->data);
}

void chatter8(const std_msgs::Int64::ConstPtr& msg)
{
  ROS_INFO("I heard: [%lli]", msg->data);
}

void chatter9(const std_msgs::Int8::ConstPtr& msg)
{
  ROS_INFO("I heard: [%i]", msg->data);
}

void chatter10(const std_msgs::Time::ConstPtr& msg)
{
  ROS_INFO("I heard: sec=[%i] nsec=[%i]", msg->data.sec, msg->data.nsec);
}

void chatter11(const std_msgs::UInt16::ConstPtr& msg)
{
  ROS_INFO("I heard: [%i]", msg->data);
}

void chatter12(const std_msgs::UInt32::ConstPtr& msg)
{
  ROS_INFO("I heard: [%i]", msg->data);
}

void chatter13(const std_msgs::UInt64::ConstPtr& msg)
{
  ROS_INFO("I heard: [%lli]", msg->data);
}

void chatter14(const std_msgs::UInt8::ConstPtr& msg)
{
  ROS_INFO("I heard: [%i]", msg->data);
}

void chatter15(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->data);
}

void chatter16(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "std_msgs_listener");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("bool", 1000, chatter1);
  ros::Subscriber sub2 = n.subscribe("byte", 1000, chatter2);
  ros::Subscriber sub3 = n.subscribe("char", 1000, chatter3);
  ros::Subscriber sub4 = n.subscribe("duration", 1000, chatter4);
  ros::Subscriber sub5 = n.subscribe("header", 1000, chatter5);
  ros::Subscriber sub6 = n.subscribe("/int16", 1000, chatter6);
  ros::Subscriber sub7 = n.subscribe("/int32", 1000, chatter7);
  ros::Subscriber sub8 = n.subscribe("/int64", 1000, chatter8);
  ros::Subscriber sub9 = n.subscribe("/int8", 1000, chatter9);
  ros::Subscriber sub10 = n.subscribe("/time", 1000, chatter10);
  ros::Subscriber sub11 = n.subscribe("/uint16", 1000, chatter11);
  ros::Subscriber sub12 = n.subscribe("/uint32", 1000, chatter12);
  ros::Subscriber sub13 = n.subscribe("/uint64", 1000, chatter13);
  ros::Subscriber sub14 = n.subscribe("/uint8", 1000, chatter14);
  ros::Subscriber sub15 = n.subscribe("/float32", 1000, chatter15);
  ros::Subscriber sub16 = n.subscribe("/float64", 1000, chatter16);

  ros::spin();

  return 0;
}

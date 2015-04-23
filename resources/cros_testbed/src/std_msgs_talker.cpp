#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Char.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "std_msgs_talker");
  ros::NodeHandle n;
  ros::Publisher chatter1 = n.advertise<std_msgs::Bool>("bool", 1000);
  ros::Publisher chatter2 = n.advertise<std_msgs::Byte>("byte", 1000);
  ros::Publisher chatter3 = n.advertise<std_msgs::Char>("char", 1000);
  ros::Publisher chatter4 = n.advertise<std_msgs::Duration>("duration", 1000);
  ros::Publisher chatter5 = n.advertise<std_msgs::Header>("header", 1000);
  ros::Publisher chatter6 = n.advertise<std_msgs::Int16>("int16", 1000);
  ros::Publisher chatter7 = n.advertise<std_msgs::Int32>("int32", 1000);
  ros::Publisher chatter8 = n.advertise<std_msgs::Int64>("int64", 1000);
  ros::Publisher chatter9 = n.advertise<std_msgs::Int8>("int8", 1000);
  ros::Publisher chatter10 = n.advertise<std_msgs::Time>("time", 1000);
  ros::Publisher chatter11 = n.advertise<std_msgs::UInt16>("uint16", 1000);
  ros::Publisher chatter12 = n.advertise<std_msgs::UInt32>("uint32", 1000);
  ros::Publisher chatter13 = n.advertise<std_msgs::UInt64>("uint64", 1000);
  ros::Publisher chatter14 = n.advertise<std_msgs::UInt8>("uint8", 1000);
  ros::Publisher chatter15 = n.advertise<std_msgs::Float32>("float32", 1000);
  ros::Publisher chatter16 = n.advertise<std_msgs::Float64>("float64", 1000);
  ros::Rate loop_rate(1);
  std::cout<<ros::XMLRPCManager::instance()->getServerURI()<<std::endl;

  int count = 0;
  while (ros::ok())
  {
    std_msgs::Bool msg1;
    msg1.data = true;
    chatter1.publish(msg1);

    std_msgs::Byte msg2;
    msg2.data = -3;
    chatter2.publish(msg2);

    std_msgs::Char msg3;
    msg3.data = 'a';
    chatter3.publish(msg3);

    std_msgs::Duration msg4;
    msg4.data.sec = 2;
    msg4.data.nsec = 3;
    chatter4.publish(msg4);

    std_msgs::Header msg5;
    msg5.seq = 2;
    msg5.stamp.sec = 6;
    msg5.stamp.nsec = 21;
    msg5.frame_id = "Ciao";
    chatter5.publish(msg5);

    std_msgs::Int16 msg6;
    msg6.data = -1024;
    chatter6.publish(msg6);

    std_msgs::Int32 msg7;
    msg7.data = -10000000;
    chatter7.publish(msg7);

    std_msgs::Int64 msg8;
    msg8.data = -10000000001;
    chatter8.publish(msg8);

    std_msgs::Int8 msg9;
    msg9.data = -5;
    chatter9.publish(msg9);

    std_msgs::Time msg10;
    msg10.data.sec = 4;
    msg10.data.nsec = 12;
    chatter10.publish(msg10);

    std_msgs::UInt16 msg11;
    msg11.data = 1024;
    chatter11.publish(msg11);

    std_msgs::UInt32 msg12;
    msg12.data = 10000000;
    chatter12.publish(msg12);

    std_msgs::UInt64 msg13;
    msg13.data = 10000000001;
    chatter13.publish(msg13);

    std_msgs::UInt8 msg14;
    msg14.data = 5;
    chatter14.publish(msg14);

    std_msgs::Float32 msg15;
    msg15.data = 0.3f;
    chatter15.publish(msg15);

    std_msgs::Float64 msg16;
    msg16.data = 0.5;
    chatter16.publish(msg16);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

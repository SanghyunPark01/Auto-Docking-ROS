#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Test1_marker");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("/check_DockingSystem_Flag", 10);
  ros::Rate loop_rate(100);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::Int32 msg;

    msg.data = 1;

    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
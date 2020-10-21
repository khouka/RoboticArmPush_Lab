#include <ros/ros.h>
#include <std_msgs/Float64.h>
// Other important libraries

int main(int argc, char **argv)
{
  ros::init(argc, argv, "< node name>");
  ros::NodeHandle n;
  ros::Publisher joint1_pub;
  joint1_pub = n.advertise<std_msgs::Float64>("< ROS topic name", 10); 
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::Float64 <message name>;
    <message name>.data = 2.0;
    joint1_pub.publish(<message name>);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

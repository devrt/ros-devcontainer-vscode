#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_name");
  ros::NodeHandle n;
  std_msgs::String msg;
  ros::Publisher pub = n.advertise<std_msgs::String>("topic_name", 1000);
  ros::Rate rate(10);

  while (ros::ok())
  {
    msg.data = "";
    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

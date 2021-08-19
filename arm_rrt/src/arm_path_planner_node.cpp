#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "arm_path_planner");  // initializing the node
  ros::NodeHandle n;

  // all the publihers defined here
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


  //all the subscriber defined here
  ros::Subscriber sub = n.subscribe("chatter", 1000, );




  //logic starts
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
	chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
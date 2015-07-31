#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"

ros::Publisher publisher;
geometry_msgs::Polygon square;


void CallbackKeyframeMsg(const geometry_msgs::Polygon square)
{

    // ROS_INFO("Suca");

    ROS_INFO("Square: 3 Point (%f , %f) ", square.points[2].x, square.points[2].y);
     
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "reconstruction");

  ros::NodeHandle n;

  ros::Subscriber kfsub = n.subscribe("/square", 1, CallbackKeyframeMsg);

  ros::spin();

  return 0;
}

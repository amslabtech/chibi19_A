#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"
#include <sstream>

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
//	ROS_INFO("%f",);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roomba_auto_test");
 
  ros::NodeHandle n;

  ros::Subscriber roomba_auto_sub = n.subscribe("roomba/odometry", 1,chatterCallback);

  ros::Publisher roomba_auto_pub = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    roomba_500driver_meiji::RoombaCtrl roomba_auto;
    nav_msgs::Odometry roomba_odom;



    roomba_auto.mode = 11;
    roomba_auto.cntl.linear.x = 1.0f;
    // std::stringstream ss;
    // ss << "hello world" << count;
    // roomba_auto.data = ss.str();

    ROS_INFO("%f", roomba_auto.cntl.linear.x);

    roomba_auto_pub.publish(roomba_auto);

    if(roomba_odom.pose.pose.position.x == 3.0f){
	    while(roomba_odom.pose.pose.orientation.z < 2.0f){
		    roomba_auto.cntl.angular.z = 1.0f;
	    }
    }



    ros::spinOnce();

    loop_rate.sleep();
    }
  return 0;
}

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roomba_auto_test");
  ros::NodeHandle n;
  ros::Publisher roomba_auto_pub = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    roomba_500driver_meiji::RoombaCtrl roomba_auto;

    roomba_auto.mode = 11;
    roomba_auto.cntl.linear.x = 1.0f;
    // std::stringstream ss;
    // ss << "hello world" << count;
    // roomba_auto.data = ss.str();

    ROS_INFO("%f", roomba_auto.cntl.linear.x);

    roomba_auto_pub.publish(roomba_auto);

    ros::spinOnce();

    loop_rate.sleep();
    }
  return 0;
}

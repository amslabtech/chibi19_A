#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include <sstream>
#include <cmath>
#include <vector>
#include <limits>

//パラメータの調整が必要
#define robot_radius 0.2f
#define max_speed 1.0f
#define min_speed -1.0f
#define max_yawrate 1.0f
#define max_accel 1.0f
#define max_dyawrate 1.0f
#define dv 0.01f
#define dyaw 0.01f
#define dt 0.1f
#define predict_time 3.0f
#define to_goal_cost_gain 1.0f
#define speed_cost_gain 1.0f
#define ob_cost_gain 1.0f

nav_msgs::Odometry roomba_odom;
sensor_msgs::LaserScan roomba_scan;

void motion(float x[], const float u[])
{
  x[2] += u[1] * dt;
  x[0] += u[0] * cos(x[2]) * dt;
  x[1] += u[0] * sin(x[2]) * dt;
  x[3] = u[0];
  x[4] = u[1];
}

void calc_dynamic_window(float dw[], const float x[])
{
  float Vs[] = {
    min_speed,
    max_speed,
    -max_yawrate,
    max_yawrate
  };

  float Vd[] = {
    x[3] - max_accel * dt,
    x[3] + max_accel * dt,
    x[4] - max_dyawrate * dt,
    x[4] + max_dyawrate * dt
  };

  dw[0] = std::max(Vs[0], Vd[0]);
  dw[1] = std::min(Vs[1], Vd[1]);
  dw[2] = std::max(Vs[2], Vd[2]);
  dw[3] = std::min(Vs[3], Vd[3]);
}

void calc_trajectory(std::vector<float*> &traj, const float xinit[], const float v, const float y)
{
  float u[] = {v, y};
  float x[] = {
    xinit[0],
    xinit[1],
    xinit[2],
    xinit[3],
    xinit[4]
  };

  for(float t = 0; t <= predict_time; t += dt){
    traj.push_back(x);
    motion(x, u);
  }
  traj.push_back(x);
  ROS_INFO("calc_traj traj.back()[0] = %f\n", traj.back()[0]);
  ROS_INFO("calc_traj traj.size() = %ld\n", traj.size());
}

float calc_to_goal_cost(const std::vector<float*> &traj, const float goal[])
{
  ROS_INFO("calc_to_goal_cost in\n");
  ROS_INFO("calc_to_goal_cost %ld\n", traj.size());
  float *p = traj.back();
  float error_angle = std::abs(std::atan2(goal[1] - p[1], goal[0] - p[0]) - p[2]);

  return to_goal_cost_gain * (M_PI - error_angle);
}

float calc_speed_cost(const std::vector<float*> &traj)
{
  float *p = traj.back();
  float error_speed = max_speed - p[3];

  return speed_cost_gain * error_speed;
}

float calc_obstacle_cost(const std::vector<float*> &traj, std::vector<float> ob)
{
  int skip_i = 2;
  float *local_origin = traj.front();
  float local_x = 0.0f;
  float local_y = 0.0f;
  float local_r = 0.0f;
  float local_theta = 0.0f;
  float ob_theta = 0.0f;
  float dist = 0.0f;
  float min_dist = std::numeric_limits<float>::infinity();

  for(int i = 0; i < traj.size(); i += skip_i){
    local_x = traj[i][0] - local_origin[0];
    local_y = traj[i][1] - local_origin[1];
    local_r = std::sqrt(local_x*local_x + local_y*local_y);
    local_theta = std::atan2(local_y, local_x);
    ob_theta = roomba_scan.angle_min;

    for(int j = 0; j < ob.size(); j++){
      //極座標での２点間の距離を調べる
      dist = std::sqrt(local_r*local_r + ob[j]*ob[j] - \
          2*local_r*ob[j]*cos(local_theta - ob_theta));

      ROS_INFO("dist = %f\n", dist);
      if(dist <= robot_radius){
        return std::numeric_limits<float>::infinity();
        //return 0.0f;
      }

      if(min_dist >= dist){
        min_dist = dist;
      }
      
      ob_theta += roomba_scan.angle_increment;
    }
  }

      ROS_INFO("ob_cost_gain / min_dist = %f\n", ob_cost_gain / min_dist);
  return ob_cost_gain / min_dist;
}

void dwa_control(float output_u[], const float x[], const float goal[], const std::vector<float> ob)
{
  float dw[4] = {0.0f};
  float best_u[2] = {0.0f};
  float min_cost = 10000.0f;
  float to_goal_cost = 0.0f;
  float speed_cost = 0.0f;
  float ob_cost = 0.0f;
  float final_cost = 0.0f;
  std::vector<float*> traj;

  calc_dynamic_window(dw, x);

  for(float v = dw[0]; v <= dw[1]; v += dv){
    for(float y = dw[2]; y <= dw[3]; y += dyaw){
      calc_trajectory(traj, x, v, y);
  ROS_INFO("dwa traj.size() = %ld\n", traj.size());
  ROS_INFO("calc_trajectory ok\n");

      to_goal_cost = calc_to_goal_cost(traj, goal);
  ROS_INFO("to_goal_cost = %f\n", to_goal_cost);
      speed_cost = calc_speed_cost(traj);
  ROS_INFO("speed_cost = %f\n", speed_cost);
      ob_cost = calc_obstacle_cost(traj, ob);
  ROS_INFO("ob_cost = %f\n", ob_cost);

      final_cost = to_goal_cost + speed_cost + ob_cost;

      if(min_cost >= final_cost){
        min_cost = final_cost;
        best_u[0] = v;
        best_u[1] = y;
  ROS_INFO("v = %f\n", v);
  ROS_INFO("y = %f\n", y);
      }
    }
  }

  output_u[0] = best_u[0];
  output_u[1] = best_u[1];
}

int is_goal(const float x[], const float goal[])
{
  float dx = goal[0] - x[0];
  float dy = goal[1] - x[1];

  float dist = std::sqrt(dx*dx + dy*dy);

  if(dist <= robot_radius){
    return 0;
  } else {
    return 11;
  }
}

int is_normalized()
{
  double square = roomba_odom.pose.pose.orientation.x *\
      roomba_odom.pose.pose.orientation.x +\
      roomba_odom.pose.pose.orientation.y *\
      roomba_odom.pose.pose.orientation.y +\
      roomba_odom.pose.pose.orientation.z *\
      roomba_odom.pose.pose.orientation.z +\
      roomba_odom.pose.pose.orientation.w *\
      roomba_odom.pose.pose.orientation.w;
  
  if(std::fabs(square - 1.0) > 0.1){
  ROS_INFO("square = %lf\n",square);
    return 1;
  } else {
  ROS_INFO("square = %lf\n",square);
    return 0;
  }
}

void chatter_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  roomba_odom = *msg;
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  roomba_scan = *scan_msg;
}

int main(int argc, char **argv)
{
  int roomba_mode = 11;

  //goal position [x(m), y(m)]
  //後ほどまた設定する
  float goal[] = {10000.0f, 100000.0f};

  ros::init(argc, argv, "dwa");
  ros::NodeHandle n;

  ros::Publisher roomba_auto_pub = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
  ros::Subscriber roomba_odom_sub = n.subscribe("roomba/odometry", 1,chatter_callback);
  ros::Subscriber roomba_scan_sub = n.subscribe("scan",1,scan_callback);

  ros::Rate loop_rate(10);

  ROS_INFO("start");
  
  while (ros::ok())
  {
    ros::spinOnce();

    if(!roomba_scan.ranges.size() || is_normalized()){
      continue;
    }

    float x[] = {
      (float)roomba_odom.pose.pose.position.x,
      (float)roomba_odom.pose.pose.position.y,
      (float)tf::getYaw(roomba_odom.pose.pose.orientation),
      (float)roomba_odom.twist.twist.linear.x,
      (float)roomba_odom.twist.twist.angular.z
    };

    //ゴール判別
    roomba_mode = is_goal(x, goal);

    float output_u[2];
  ROS_INFO("%ld\n", roomba_scan.ranges.size());
    dwa_control(output_u, x, goal, roomba_scan.ranges);

  ROS_INFO("dwa ok");
    roomba_500driver_meiji::RoombaCtrl roomba_auto;

  ROS_INFO("output_u[0] = %f\n", output_u[0]);
  ROS_INFO("output_u[1] = %f\n", output_u[1]);
    roomba_auto.mode = roomba_mode;
    roomba_auto.cntl.linear.x = output_u[0];
    roomba_auto.cntl.angular.z = output_u[1];

    roomba_auto_pub.publish(roomba_auto);

    loop_rate.sleep();
  }

  return 0;
}

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <cmath>
#include <vector>
#include <limits>

//パラメータの調整が必要
#define robot_radius 1.0f
#define max_speed 1.0f
#define min_speed -0.5f
#define max_yawrate 40.0f * (float)M_PI / 180.0f
#define max_accel 0.2f
#define max_dyawrate 40.0f * (float)M_PI / 180.0f
#define dv 0.01f
#define dyaw 0.1f * (float)M_PI / 180.0f
#define dt 0.1f
#define predict_time 3.0f
#define to_goal_cost_gain 1.0f
#define speed_cost_gain 1.0f
#define ob_cost_gain 1.0f

nav_msgs::Odometry roomba_odom;
sensor_msgs::LaserScan roomba_scan;

//int roomba_mode = 11;
//bool flag = true;

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

  /*
  dw = {
    std::max(Vs[0], Vd[0]),
    std::min(Vs[1], Vd[1]),
    std::max(Vs[2], Vd[2]),
    std::min(Vs[3], Vd[3])
  };*/

  dw[0] = std::max(Vs[0], Vd[0]);
  dw[1] = std::min(Vs[1], Vd[1]);
  dw[2] = std::max(Vs[2], Vd[2]);
  dw[3] = std::min(Vs[3], Vd[3]);
  //return dw;
}

void calc_trajectory(std::vector<float*> traj, const float xinit[], const float v, const float y)
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
  /*
  time = 0
  while(time <= config.predict_time){
    x = motion(x, [v, y], config.dt);
    traj = np.vstack((traj, x));
    time += config.dt;
  }*/

  //return traj;
}
/*
void calc_final_input(float output_u[], const float x[], const float dw[], const float goal[], const float ob[])
{
  for v in np.arange(dw[0], dw[1], config.dv){
    for y in np.arange(dw[2], dw[3], config.dy){
      traj = calc_trajectory(xinit, v, y, config);

      to_goal_cost = calc_to_goal_cost(traj, goal, config);
      speed_cost = config.speed_cost_gain * (config.max_speed - traj[-1, 3]);
      ob_cost = calc_obstacle_cost(traj, ob, config);

      final_cost = to_goal_cost + speed_cost + ob_cost;

      if(min_cost >= final_cost){
        min_cost = final_cost;
        min_u = [v, y];
        best_traj = traj;
      }
    }
  }

  return min_u, best_traj;
}
*/

float calc_to_goal_cost(const std::vector<float*> traj, const float goal[])
{
  /*
  goal_magnitude = std::sqrt(goal[0]*goal[0]+goal[1]*goal[1]);
  traj_magnitude = std::sqrt(traj[-1, 0]*traj[-1, 0]+traj[-1, 1]*traj[-1, 1]);
  dot_product = goal[0]*traj[-1, 0] + goal[1]*traj[-1, 1];
  error = dot_product / (goal_magnitude*traj_magnitude);
  error_angle = math.acos(error);
  cost = config.to_goal_cost_gain * error_angle;*/
  float *p = traj.back();
  float error_angle = std::abs(std::atan2(goal[1] - p[1], goal[0] - p[0]) - p[2]);

  return to_goal_cost_gain * (M_PI - error_angle);
}

float calc_speed_cost(const std::vector<float*> traj)
{
  float *p = traj.back();
  float error_speed = max_speed - p[3];

  return speed_cost_gain * error_speed;
}

float calc_obstacle_cost(const std::vector<float*> traj, std::vector<float> ob)
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

      if(dist <=robot_radius){
        return std::numeric_limits<float>::infinity();
      }

      if(min_dist >= dist){
        min_dist = dist;
      }
      
      ob_theta += roomba_scan.angle_increment;
    }
  }

  return ob_cost_gain / min_dist;
  /*
  for i in range(0, len(traj[:, 1]), skip_n){
    for j in range(len(ob[:, 0])){
      ox = ob[j, 0];
      oy = ob[j, 1];
      dx = traj[i, 0] - ox;
      dy = traj[i, 1] - oy;

      r = math.sqrt(dx*dx + dy*dy);
      if (r <= config.robot_radius){
        return float("Inf");
      }

      if (minr >= r){
        minr = r;
      }

      return 1.0 / minr;
    }
  }*/
}

void dwa_control(float output_u[], const float x[], const float goal[], const std::vector<float> ob)
{
  //float u[3] = {0.0f};
  float dw[4] = {0.0f};
  calc_dynamic_window(dw, x);

  //calc_final_input(output_u, x, dw, goal, ob);
  //float min_u[] = {0.0f, x[4]};
  float min_cost = 10000.0f;
  float to_goal_cost = 0.0f;
  float speed_cost = 0.0f;
  float ob_cost = 0.0f;
  float final_cost = 0.0f;
  //min_u[0] = 0.0;
  //std::vector<float*> best_traj = x;
  std::vector<float*> traj;

  for(int v = dw[0]; v <= dw[1]; v += dv){
    for(int y = dw[2]; y <= dw[3]; y += dyaw){
      calc_trajectory(traj, x, v, y);

      to_goal_cost = calc_to_goal_cost(traj, goal);
      speed_cost = calc_speed_cost(traj);
      ob_cost = calc_obstacle_cost(traj, ob);

      final_cost = to_goal_cost + speed_cost + ob_cost;

      if(min_cost >= final_cost){
        //min_cost = final_cost;
        output_u[0] = v;
        output_u[1] = y;
      }
    }
  }
  /*
  for v in np.arange(dw[0], dw[1], config.dv){
    for y in np.arange(dw[2], dw[3], config.dy){
      traj = calc_trajectory(xinit, v, y, config);

      to_goal_cost = calc_to_goal_cost(traj, goal, config);
      speed_cost = config.speed_cost_gain * (config.max_speed - traj[-1, 3]);
      ob_cost = calc_obstacle_cost(traj, ob, config);

      final_cost = to_goal_cost + speed_cost + ob_cost;

      if(min_cost >= final_cost){
        min_cost = final_cost;
        min_u = [v, y];
        best_traj = traj;
      }
    }
  }*/

  //return u;
  //, best_traj;
}

int is_goal(const float x[], const float goal[])
{
  float dx = goal[0] - x[0];
  float dy = goal[1] - x[1];

  float dist = std::sqrt(dx*dx + dy*dy);

  if(dist <=robot_radius){
    return 0;
  } else {
    return 11;
  }
}

void chatter_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  roomba_odom = *msg;
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  roomba_scan = *scan_msg;
  /*
  int front_scan_pos = roomba_scan.ranges.size()/2;

  if(front_scan_pos && roomba_scan.ranges[front_scan_pos] < 0.5 && !flag){
    roomba_mode = 0;
  }*/
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

    if(!roomba_scan.ranges.size()){
      continue;
      ROS_INFO("roomba_scan.ranges.size()=0");
    }

    float u = std::sqrt(
      (float)roomba_odom.twist.twist.linear.x*\
      (float)roomba_odom.twist.twist.linear.x +\
      (float)roomba_odom.twist.twist.linear.y*\
      (float)roomba_odom.twist.twist.linear.y
    );

    //roomba_odom.pose.pose.orientation.xは絶対座標だったので書き直し
    float x[] = {
      (float)roomba_odom.pose.pose.position.x,
      (float)roomba_odom.pose.pose.position.y,
      (float)roomba_odom.pose.pose.orientation.z,
      u,//(float)roomba_odom.twist.twist.linear.x,
      (float)roomba_odom.twist.twist.angular.z
    };

    //ゴール判別
    roomba_mode = is_goal(x, goal);

    //float ob[] = roomba_scan.ranges;
        /**std::min_element(
              roomba_scan.ranges.begin(),
              roomba_scan.ranges.end()
            );*/

    float output_u[3];
    dwa_control(output_u, x, goal, roomba_scan.ranges);

    roomba_500driver_meiji::RoombaCtrl roomba_auto;

    roomba_auto.mode = roomba_mode;
    roomba_auto.cntl.linear.x = output_u[0];
    //roomba_auto.cntl.linear.y = output_u[1];
    roomba_auto.cntl.angular.z = output_u[1];

    roomba_auto_pub.publish(roomba_auto);

    loop_rate.sleep();
  }

  return 0;
}

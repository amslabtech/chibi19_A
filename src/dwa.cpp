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
#define min_speed 0.0f
#define max_yawrate 1.0f
#define max_accel 3.0f
#define max_dyawrate 5.0f
#define dv 0.01f
#define dyaw 0.01f
#define dt 0.1f
#define predict_time 1.5f
#define to_goal_cost_gain 0.001f
#define speed_cost_gain 1.00f
#define ob_cost_gain 1.80f

nav_msgs::Odometry roomba_odom;
sensor_msgs::LaserScan roomba_scan;

void motion(std::vector<float>& x, const std::vector<float>& u)
{
  x[2] += u[1] * dt;
  if(x[2] > M_PI){
	  x[2] -= 2*M_PI;
  }
  x[0] += u[0] * cos(x[2]) * dt;
  x[1] += u[0] * sin(x[2]) * dt;
  x[3] = u[0];
  x[4] = u[1];
}

void calc_dynamic_window(std::vector<float>& dw, const std::vector<float>& x)
{
  //ROS_INFO("calc_dynamic_window in\n");
  std::vector<float> Vs= {
    min_speed,
    max_speed,
    -max_yawrate,
    max_yawrate
  };
  //ROS_INFO("calc_dynamic_window Vs[0] = %f\n", Vs[0]);
  //ROS_INFO("calc_dynamic_window Vs[1] = %f\n", Vs[1]);
  //ROS_INFO("calc_dynamic_window Vs[2] = %f\n", Vs[2]);
  //ROS_INFO("calc_dynamic_window Vs[3] = %f\n", Vs[3]);

  std::vector<float> Vd = {
    x[3] - max_accel * dt,
    x[3] + max_accel * dt,
    x[4] - max_dyawrate * dt,
    x[4] + max_dyawrate * dt
  };
  //ROS_INFO("calc_dynamic_window Vd[0] = %f\n", Vd[0]);
  //ROS_INFO("calc_dynamic_window Vd[1] = %f\n", Vd[1]);
  //ROS_INFO("calc_dynamic_window Vd[2] = %f\n", Vd[2]);
  //ROS_INFO("calc_dynamic_window Vd[3] = %f\n", Vd[3]);

  dw[0] = std::max(Vs[0], Vd[0]);
  dw[1] = std::min(Vs[1], Vd[1]);
  dw[2] = std::max(Vs[2], Vd[2]);
  dw[3] = std::min(Vs[3], Vd[3]);
 //ROS_INFO("calc_dynamic_window dw[0] = %f\n", dw[0]);
 // ROS_INFO("calc_dynamic_window dw[1] = %f\n", dw[1]);
 // ROS_INFO("calc_dynamic_window dw[2] = %f\n", dw[2]);
 // ROS_INFO("calc_dynamic_window dw[3] = %f\n", dw[3]);
}

void calc_trajectory(std::vector<std::vector<float> >& traj, const std::vector<float>& xinit, const float v, const float y)
{
  //ROS_INFO("calc_trajectory in\n");
  //ROS_INFO("calc_trajectory v = %f\n", v);
  //ROS_INFO("calc_trajectory y = %f\n", y);
	std::vector<float> u = {v, y};

	std::vector<float> x = {
    xinit[0],
    xinit[1],
    xinit[2],
    xinit[3],
    xinit[4]
  };
    //ROS_INFO("calc_trajectory x[0] = %f\n", x[0]);
    //ROS_INFO("calc_trajectory x[1] = %f\n", x[1]);
    //ROS_INFO("calc_trajectory x[2] = %f\n", x[2]);
    //ROS_INFO("calc_trajectory x[3] = %f\n", x[3]);
    //ROS_INFO("calc_trajectory x[4] = %f\n", x[4]);

  traj.erase(traj.begin(), traj.end());
  for(float t = 0; t <= predict_time; t += dt){
    traj.push_back(x);
    motion(x, u);
//    ROS_INFO("calc_trajectory    t = %f\n", t);
//    ROS_INFO("calc_trajectory x[0] = %f\n", x[0]);
//    ROS_INFO("calc_trajectory x[1] = %f\n", x[1]);
//    ROS_INFO("calc_trajectory x[2] = %f\n", x[2]);
//    ROS_INFO("calc_trajectory x[3] = %f\n", x[3]);
//    ROS_INFO("calc_trajectory x[4] = %f\n", x[4]);
  }
  traj.push_back(x);
  //ROS_INFO("calc_trajectory traj.back()[0] = %f\n", traj.back()[0]);
  //ROS_INFO("calc_trajectory traj.size() = %ld\n", traj.size());
    //ROS_INFO("calc_trajectory traj.back()[0] = %f\n", traj.back()[0]);
    //ROS_INFO("calc_trajectory traj.back()[1] = %f\n", traj.back()[1]);
    //ROS_INFO("calc_trajectory traj.back()[2] = %f\n", traj.back()[2]);
    //ROS_INFO("calc_trajectory traj.back()[3] = %f\n", traj.back()[3]);
    //ROS_INFO("calc_trajectory traj.back()[4] = %f\n", traj.back()[4]);
}

float calc_to_goal_cost(std::vector<std::vector<float> >& traj, const std::vector<float>& goal)
{
  //ROS_INFO("calc_to_goal_cost in\n");
  //ROS_INFO("calc_to_goal_cost %ld\n", traj.size());
	std::vector<float> p = traj.back();
    //ROS_INFO("calc_to_goal_cost p[0] = %f\n", p[0]);
    //ROS_INFO("calc_to_goal_cost p[1] = %f\n", p[1]);
    //ROS_INFO("calc_to_goal_cost p[2] = %f\n", p[2]);
    //ROS_INFO("calc_to_goal_cost p[3] = %f\n", p[3]);
    //ROS_INFO("calc_to_goal_cost p[4] = %f\n\n", p[4]);
  float to_goal_angle = std::atan2(goal[1] - p[1], goal[0] -p[0]);

  if(to_goal_angle > M_PI){
	  to_goal_angle -= 2*M_PI;
  }
  float error_angle = std::abs(to_goal_angle - p[2]);
    //ROS_INFO("calc_to_goal_cost error_angle = %f\n\n", error_angle);

  return to_goal_cost_gain * error_angle;
}

float calc_speed_cost(std::vector<std::vector<float> >& traj)
{
  std::vector<float> p = traj.back();
  //ROS_INFO("calc_speed_cost p[0] = %f\n", p[0]);
  //ROS_INFO("calc_speed_cost p[1] = %f\n", p[1]);
  //ROS_INFO("calc_speed_cost p[2] = %f\n", p[2]);
  //ROS_INFO("calc_speed_cost p[3] = %f\n", p[3]);
  //ROS_INFO("calc_speed_cost p[4] = %f\n", p[4]);
  float error_speed = max_speed - p[3];

  return speed_cost_gain * error_speed;
}

float calc_obstacle_cost(std::vector<std::vector<float> >& traj, std::vector<float> ob)
{
  int skip_i = 3;
  int skip_j = 1;
  std::vector<float> local_origin = traj.front();
  //float local_origin_x = traj.front()[0];
  //float local_origin_y = traj.front()[1]; 
  //ROS_INFO("local_origin[0] = %f\n", local_origin[0]);
  //ROS_INFO("local_origin[1] = %f\n", local_origin[1]);
  float min_dist = std::numeric_limits<float>::infinity();
  float local_x = 0.0f;
  float local_y = 0.0f;
  float local_r = 0.0f;
  float local_theta = 0.0f;
  float ob_theta = 0.0f;
  float dist = 0.0f;

  //ROS_INFO("traj.size() = %ld\n\n", traj.size());
  for(int i = 0; i < traj.size(); i += skip_i){
	dist = 0.0f;

    local_x = traj[i][0] - local_origin[0];
      //ROS_INFO("local_x = %f\n", local_x);
    local_y = traj[i][1] - local_origin[1];
      //ROS_INFO("local_y = %f\n\n", local_y);
    local_r = std::sqrt(local_x*local_x + local_y*local_y);
      //ROS_INFO("local_r = %f\n", dist);
	if(!local_x){
		local_theta = 0.0f;
	} else {
    	local_theta = std::atan2(local_y, local_x);
	}
    ob_theta = roomba_scan.angle_min;

      ROS_INFO("local_theta = %f\n", local_theta);
      //ROS_INFO("ob_theta = %f\n", ob_theta);
	  //ROS_INFO("i = %d \n\n", i);

    for(int j = 0; j < ob.size(); j += skip_j){
	  if(ob[j] <= robot_radius || ob[j] > 60.0f){
		  continue;
	  }
      //極座標での２点間の距離を調べる
      dist = std::sqrt(local_r*local_r + ob[j]*ob[j] - \
          2*local_r*ob[j]*cos(local_theta - ob_theta));
 
	  if(dist <= robot_radius){
	//    ROS_INFO("\nj = %d \n", j);
	//  ROS_INFO("local_r = %f\n", local_r);
	//  ROS_INFO("ob[j] = %f\n", ob[j]);
	//  ROS_INFO("local_theta = %f\n", local_theta);
	//  ROS_INFO("ob_theta = %f\n", ob_theta);
	//    ROS_INFO("dist = %f\n", dist);
        return std::numeric_limits<float>::infinity();
        //return 0.0f;
      }

      if(min_dist >= dist){
        min_dist = dist;
      }
      
      ob_theta += roomba_scan.angle_increment;
    }
  }

  return ob_cost_gain / min_dist;
}

bool dwa_control(std::vector<float>& output_u, const std::vector<float>& x, const std::vector<float>& goal, const std::vector<float>& ob)
{
  std::vector<float> dw = {0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> best_u = {0.0f, 0.0f};
  std::vector<std::vector<float> > traj;
  float min_cost = 10000.0f;
  float to_goal_cost = 0.0f;
  float speed_cost = 0.0f;
  float ob_cost = 0.0f;
  float final_cost = 0.0f;

  calc_dynamic_window(dw, x);
  ROS_INFO("dwa dw[0] = %f\n", dw[0]);
  ROS_INFO("dwa dw[1] = %f\n", dw[1]);
  ROS_INFO("dwa dw[2] = %f\n", dw[2]);
  ROS_INFO("dwa dw[3] = %f\n", dw[3]);

  for(float v = dw[0]; v <= dw[1]; v += dv){
    for(float y = dw[2]; y <= dw[3]; y += dyaw){
  //ROS_INFO("dw[0] = %f\n", dw[0]);
  //ROS_INFO("dw[1] = %f\n", dw[1]);
  //ROS_INFO("dw[2] = %f\n", dw[2]);
  //ROS_INFO("dw[3] = %f\n", dw[3]);
      calc_trajectory(traj, x, v, y);
    //ROS_INFO("dwa_control traj.back()[0] = %f\n", traj.back()[0]);
    //ROS_INFO("dwa_control traj.back()[1] = %f\n", traj.back()[1]);
    //ROS_INFO("dwa_control traj.back()[2] = %f\n", traj.back()[2]);
    //ROS_INFO("dwa_control traj.back()[3] = %f\n", traj.back()[3]);
    //ROS_INFO("dwa_control traj.back()[4] = %f\n", traj.back()[4]);
  //ROS_INFO("dwa traj.size() = %ld\n", traj.size());
  //ROS_INFO("calc_trajectory ok\n");

  //ROS_INFO("v = %f\n", v);
      to_goal_cost = calc_to_goal_cost(traj, goal);
  //ROS_INFO("to_goal_cost = %f\n", to_goal_cost);
      speed_cost = calc_speed_cost(traj);
  //ROS_INFO("speed_cost = %f\n", speed_cost);
      ob_cost = calc_obstacle_cost(traj, ob);
  //ROS_INFO("ob_cost = %f\n\n", ob_cost);

      final_cost = to_goal_cost + speed_cost + ob_cost;

  //ROS_INFO("v = %f\nfinal_cost = %f\n\n", v, final_cost);

  ROS_INFO("\nv = %f\n", v);
  ROS_INFO("y = %f\n", y);
  ROS_INFO("final_cost = %f\n", final_cost);
  ROS_INFO("to_goal_cost = %f\n", to_goal_cost);
  ROS_INFO("speed_cost = %f\n", speed_cost);
  ROS_INFO("ob_cost = %f\n\n", ob_cost);
      if(min_cost > final_cost){
        min_cost = final_cost;
        best_u[0] = v;
        best_u[1] = y;
  ROS_INFO("\nmincost change!!!\n\n");
  ROS_INFO("v = %f\n", v);
  ROS_INFO("mincost change v = %f\n", best_u[0]);
  ROS_INFO("mincost change y = %f\n", best_u[1]);
  ROS_INFO("min_cost = %f\n", min_cost);
  ROS_INFO("to_goal_cost = %f\n", to_goal_cost);
  ROS_INFO("speed_cost = %f\n", speed_cost);
  ROS_INFO("ob_cost = %f\n\n", ob_cost);
      }
    }
  }

  ROS_INFO("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
  ROS_INFO("v = %f\n", best_u[0]);
  ROS_INFO("y = %f\n", best_u[1]);
  output_u[0] = best_u[0];
  output_u[1] = best_u[1];
  ROS_INFO("v = %f\n", output_u[0]);
  ROS_INFO("y = %f\n", output_u[1]);

  return true;
}

int is_goal(const std::vector<float>& x, const std::vector<float>& goal)
{
  float dx = goal[0] - x[0];
  float dy = goal[1] - x[1];

  float dist = std::sqrt(dx*dx + dy*dy);

  if(dist <= robot_radius){
  	ROS_INFO("\n\nGoal!!!\n\n");
	
    return 0;
  } else {
    return 11;
  }
}

int is_normalized()
{
  double square_sum = roomba_odom.pose.pose.orientation.x *\
                      roomba_odom.pose.pose.orientation.x +\
                      roomba_odom.pose.pose.orientation.y *\
                      roomba_odom.pose.pose.orientation.y +\
                      roomba_odom.pose.pose.orientation.z *\
                      roomba_odom.pose.pose.orientation.z +\
                      roomba_odom.pose.pose.orientation.w *\
                      roomba_odom.pose.pose.orientation.w;
  
  if(std::fabs(square_sum - 1.0) > 0.1){
    return false;
  } else {
    return true;
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
  std::vector<float> goal = {10.0f, 10.0f};

  ros::init(argc, argv, "dwa");
  ros::NodeHandle n;

  ros::Publisher roomba_auto_pub = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
  ros::Subscriber roomba_odom_sub = n.subscribe("roomba/odometry", 1,chatter_callback);
  ros::Subscriber roomba_scan_sub = n.subscribe("scan",1,scan_callback);

  ros::Rate loop_rate(5);

  ROS_INFO("start");
  
//  int count = 0;
  while (ros::ok())
  {
//    ROS_INFO("roop_count = %d\n", count);
    ros::spinOnce();

    if(!roomba_scan.ranges.size() || !is_normalized()){
      continue;
    }

	std::vector<float> x = {
      (float)roomba_odom.pose.pose.position.x,
      (float)roomba_odom.pose.pose.position.y,
      (float)tf::getYaw(roomba_odom.pose.pose.orientation),
      (float)roomba_odom.twist.twist.linear.x,
      (float)roomba_odom.twist.twist.angular.z
    };
//  ROS_INFO("x[0] = %f\n", x[0]);
//  ROS_INFO("x[1] = %f\n", x[1]);
//  ROS_INFO("x[2] = %f\n", x[2]);
//  ROS_INFO("x[3] = %f\n", x[3]);
//  ROS_INFO("x[4] = %f\n", x[4]);

    //ゴール判別
    roomba_mode = is_goal(x, goal);

	std::vector<float> output_u = {0.0f, 0.0f};
  //ROS_INFO("%ld\n", roomba_scan.ranges.size());
    roomba_500driver_meiji::RoombaCtrl roomba_auto;
    if(dwa_control(output_u, x, goal, roomba_scan.ranges)){
//  ROS_INFO("dwa ok");
//  ROS_INFO("output_u[0] = %f\n", output_u[0]);
//  ROS_INFO("output_u[1] = %f\n", output_u[1]);

    roomba_auto.mode = roomba_mode;
    roomba_auto.cntl.linear.x = output_u[0];
    roomba_auto.cntl.angular.z = output_u[1];
	} else {
	  ROS_INFO("dwa not finish!!!");
      roomba_auto.mode = 0;
	}
    roomba_auto_pub.publish(roomba_auto);

//    count++;
    loop_rate.sleep();
  }

  return 0;
}

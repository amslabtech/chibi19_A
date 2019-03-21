#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include <cmath>
#include <vector>
#include <limits>
#include <sstream>

#define dt 0.1
#define dv 0.01
#define dyaw 0.01
#define max_speed 0.4
#define min_speed 0.0
#define max_accel 5.0
#define max_yawrate 1.00
#define max_dyawrate 1.5
#define predict_time 6.0
#define robot_radius 0.17
#define ob_cost_gain 1.00
#define speed_cost_gain 1.00
#define to_goal_cost_gain 0.000
#define dis_goal_cost_gain 0.010

nav_msgs::Odometry roomba_odom;
sensor_msgs::LaserScan roomba_scan;

struct Speed{
  double v;
  double omega;
};

struct Status{
  double x;
  double y;
  double yaw;
  double v;
  double omega;
};

void angle_range(double& theta)
{
  if(theta > M_PI){
    theta -= 2*M_PI;
  } else if(theta < -M_PI){
    theta += 2*M_PI;
  }
}

double atan(const double& x, const double& y)
{
  double theta = 0.0;

  if(!x){
    if(!y){
      theta = 0.0;
    }  else if(y < 0){
      theta = -1 * M_PI / 2;
    } else if(y > 0){
      theta = M_PI / 2;
    }
  } else {
    theta = std::atan2(y, x);
    angle_range(theta);
  }

  return theta;
}

void motion(Status& roomba, const Speed& u)
{
  roomba.yaw += u.omega * dt;
  angle_range(roomba.yaw);

  roomba.x += u.v * std::cos(roomba.yaw) * dt;
  roomba.y += u.v * std::sin(roomba.yaw) * dt;
  roomba.v = u.v;
  roomba.omega = u.omega;
}

void calc_dynamic_window(std::vector<double>& dw, const Status& roomba)
{
  std::vector<double> Vs= {
     min_speed,
     max_speed,
    -max_yawrate,
     max_yawrate
  };

  std::vector<double> Vd = {
    roomba.v - max_accel * dt,
    roomba.v + max_accel * dt,
    roomba.omega - max_dyawrate * dt,
    roomba.omega + max_dyawrate * dt
  };

  dw[0] = std::max(Vs[0], Vd[0]);
  dw[1] = std::min(Vs[1], Vd[1]);
  dw[2] = std::max(Vs[2], Vd[2]);
  dw[3] = std::min(Vs[3], Vd[3]);
}

void calc_trajectory(std::vector<Status>& traj, const double v, const double y)
{
  Speed u = {v, y};

  Status roomba = {0.0, 0.0, 0.0, 0.0, 0.0};

  traj.erase(traj.begin(), traj.end());
  for(double t = 0; t <= predict_time; t += dt){
    traj.push_back(roomba);
    motion(roomba, u);
  }
  traj.push_back(roomba);
}

double calc_to_goal_cost(const std::vector<Status>& traj, const Status& roomba, const Status& goal)
{
  Status last = traj.back();
  double l_r = std::sqrt(last.x*last.x + last.y*last.y);
  double l_theta = atan(last.y, last.x);
  double x = roomba.x + l_r*std::cos(roomba.yaw + l_theta);
  double y = roomba.y + l_r*std::sin(roomba.yaw + l_theta);
  double dx = goal.x - x;
  double dy = goal.y - y;
  double to_goal_angle = 0.0;
  double error_angle = 0.0;
  double error_dis = 0.0;

  to_goal_angle = atan(dx, dy);
  error_angle = std::abs(to_goal_angle - last.yaw);

  error_dis = std::sqrt(dx*dx + dy*dy);

  return to_goal_cost_gain * error_angle + dis_goal_cost_gain * error_dis;
}

double calc_speed_cost(const std::vector<Status>& traj)
{
  Status last = traj.back();
  double error_speed = max_speed - last.v;

  return speed_cost_gain * error_speed;
}

double calc_obstacle_cost(const std::vector<Status>& traj, const std::vector<float> obstacle)
{
  //x,y,r,theta,obはlocal
  int skip_i = 2;
  int skip_j = 10;
  double x = 0.0;
  double y = 0.0;
  double xx = 0.0;
  double yy = 0.0;
  double r = 0.0;
  double ob = 0.0;
  double theta = 0.0;
  double ob_theta = 0.0;
  double rr = 0.0;
  double obob = 0.0;
  double rob = 0.0;
  double c = 0.0;
  double dist = 0.0;
  double min_dist = std::numeric_limits<double>::infinity();
  double left_rod_max = 1.60;
  double left_rod_min = 0.90;
  double right_rod_max = -0.90;
  double right_rod_min = -1.60;

  for(int i = 0; i < traj.size(); i += skip_i){
    x = traj[i].x;
    y = traj[i].y;
  	xx = x*x;
  	yy = y*y;
    r = std::sqrt(xx+yy);

    theta = atan(x, y);

    ob_theta = roomba_scan.angle_min;

    for(int j = 0; j < obstacle.size(); j += skip_j){
  	  if(( left_rod_min < ob_theta && ob_theta < left_rod_max) ||
  	     ( right_rod_min < ob_theta && ob_theta < right_rod_max)){
          ob_theta += skip_j * roomba_scan.angle_increment;
          continue;
  	  }

	  if(obstacle[j] < 60.0){
		  ob = obstacle[j];
	  } else {
		  ob = 60.0;
	  }

  		  ROS_INFO("\nob_theta = %lf \n\n", ob_theta);
      }

      //極座標での２点間の距離を調べる
  	  rr = r*r;
  	  obob = ob*ob;
  	  rob = 2*r*ob;
  	  c = std::cos(theta - ob_theta);
  	  dist = std::sqrt(rr + obob - rob*c);

      if(dist <= robot_radius){
        return std::numeric_limits<double>::infinity();
      }

      if(min_dist >= dist){
        min_dist = dist;
      }

      ob_theta += skip_j * roomba_scan.angle_increment;
    }
  }

  return ob_cost_gain / min_dist;
}

Speed dwa_control(const Status& roomba, const Status& goal, const std::vector<float> ob)
{
  Speed best_output = {0.0, 0.0};
  double ob_cost = 0.0;
  double min_cost = 1000.0;
  double final_cost = 0.0;
  double speed_cost = 0.0;
  double to_goal_cost = 0.0;
  double exception_yawrate = max_yawrate * 0.09;
  std::vector<double> dw = {0.0, 0.0, 0.0, 0.0};
  std::vector<Status> traj;

  calc_dynamic_window(dw, roomba);
  for(double v = dw[0]; v <= dw[1]; v += dv){
    for(double y = dw[2]; y <= dw[3]; y += dyaw){
		if(-exception_yawrate < y && y < 0.0 ||
			0.0 < y && y < exception_yawrate){
			continue;
		}
      ROS_INFO("\n--------------------------------------------------------------------------------------------\nv = %lf\ny = %lf\n", v, y);
      calc_trajectory(traj, v, y);

      to_goal_cost = calc_to_goal_cost(traj, roomba, goal);
      speed_cost = calc_speed_cost(traj);
      ob_cost = calc_obstacle_cost(traj, ob);

      final_cost = to_goal_cost + speed_cost + ob_cost;

      ROS_INFO("\nob_cost = %lf\n\n", ob_cost);
      ROS_INFO("\nfinal_cost = %lf\n", final_cost);
      ROS_INFO("\n---------------------------------------------------------------------\nnow min_cost = %lf\n", min_cost);
      ROS_INFO("\nnow best_output.v = %lf\n", best_output.v);
      ROS_INFO("\nnow best_output.omega = %lf\n-----------------------------------------------------------------\n", best_output.omega);

      if(min_cost > final_cost){
        min_cost = final_cost;
        best_output.v = v;
        best_output.omega = y;
      }
    }
  }

  if(min_cost > 999.0){
  	best_output.v = 0.0;
  	best_output.omega = 0.3;
  }

  return best_output;
}

int is_goal(const Status& roomba, const Status& goal)
{
  double dx = goal.x - roomba.x;
  double dy = goal.y - roomba.y;

  double dist = std::sqrt(dx*dx + dy*dy);

  if(dist <= robot_radius){
    ROS_INFO("\n\nGoal!!!\n\n");
    return 0;
  } else {
    return 11;
  }
}

bool is_normalized()
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

  Speed output = {0.0, 0.0};
  Status goal = {100.0, 0.0, 0.0, 0.0, 0.0};
  Status roomba = {0.0, 0.0, 0.0, 0.0, 0.0};

  roomba_500driver_meiji::RoombaCtrl roomba_auto;

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

    if(!roomba_scan.ranges.size() || !is_normalized()){
      continue;
    }

    roomba.x = roomba_odom.pose.pose.position.x;
    roomba.y = roomba_odom.pose.pose.position.y;
    roomba.yaw = tf::getYaw(roomba_odom.pose.pose.orientation);
    roomba.v = max_speed * roomba_odom.twist.twist.linear.x;
    roomba.omega = max_yawrate * roomba_odom.twist.twist.angular.z;

    //ゴール判別
    roomba_mode = is_goal(roomba, goal);

    output = dwa_control(roomba, goal, roomba_scan.ranges);

    roomba_auto.mode = roomba_mode;
    roomba_auto.cntl.linear.x = output.v / max_speed;
    roomba_auto.cntl.angular.z = output.omega / max_yawrate;

    roomba_auto_pub.publish(roomba_auto);

    loop_rate.sleep();
  }

  return 0;
}

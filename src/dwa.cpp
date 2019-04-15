#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include <cmath>
#include <vector>
#include <limits>
#include <chrono>
#include <sstream>

nav_msgs::Path lpath;
nav_msgs::Path roomba_gpath;
nav_msgs::Odometry roomba_odom;
sensor_msgs::LaserScan roomba_scan;
geometry_msgs::PoseStamped gpath_goal;
geometry_msgs::PoseStamped roomba_status;

const double EPS = 1e-6;
bool can_goal = false;
bool get_odom = false;
bool get_pose = false;
bool line_detection;
bool get_line_detection = false;
double dt;
double dv;
double dyaw;
double stop_time;
double max_speed;
double min_speed;
double max_accel;
double ignore_line;
double limit_speed;
double max_yawrate;
double max_dyawrate;
double limit_yawrate;
double predict_time;
double roomba_radius;
double l_ob_cost_gain;
double speed_cost_gain;
double to_g_goal_cost_gain;

struct Speed{
  double v;
  double omega;
};

struct Position{
  double x;
  double y;
  double yaw;
};

struct Status{
  double x;
  double y;
  double yaw;
  double v;
  double omega;
};

struct Dw{
  double min_v;
  double max_v;
  double min_omega;
  double max_omega;
};

void calc_dynamic_window(Dw& dw, const Status& roomba)
{
  const Dw Vs= {
   min_speed,
   limit_speed,
   -limit_yawrate,
   limit_yawrate
  };
  Dw Vd = {
    roomba.v - max_accel*dt,
    roomba.v + max_accel*dt,
    roomba.omega - max_dyawrate*dt,
    roomba.omega + max_dyawrate*dt
  };

  dw.min_v = std::max(Vs.min_v, Vd.min_v);
  dw.max_v = std::min(Vs.max_v, Vd.max_v);
  dw.min_omega = std::max(Vs.min_omega, Vd.min_omega);
  dw.max_omega = std::min(Vs.max_omega, Vd.max_omega);

  return;
}

void angle_range(double& theta)
{
  if(theta > M_PI){
    theta -= 2*M_PI;
  } else if(theta < -M_PI){
    theta += 2*M_PI;
  }

  return;
}

double atan(const double& x, const double& y)
{
  double theta = 0.0;

  if(std::fabs(x) < EPS){
    if(std::fabs(y) < EPS){
      theta = 0.0;
    } else if(y < 0) {
      theta = -M_PI/2;
    } else if(y > 0) {
      theta = M_PI/2;
    }
  } else {
    theta = std::atan2(y, x);
    angle_range(theta);
  }

  return theta;
}

//二点間の距離を計算
double calc_dist(const double& x1, const double& x2, const double& y1, const double& y2){
  double dx = x1 - x2;
  double dy = y1 - y2;
  double dist = 0.0;
  
  dist = std::sqrt(dx*dx + dy*dy);

  return dist;
}

//全部local
void motion(Status& roomba, const double& v, const double& y)
{
  roomba.yaw += y*dt;
  angle_range(roomba.yaw);
  roomba.x += v*std::cos(roomba.yaw)*dt;
  roomba.y += v*std::sin(roomba.yaw)*dt;
  roomba.v = v;
  roomba.omega = y;

  return;
}

//全部local
void calc_l_traj(std::vector<Status>& traj, const double& v, const double& y)
{
  Status roomba = {0.0, 0.0, 0.0, 0.0, 0.0};

  traj.clear();
  for(double t = 0.0; t < predict_time; t += dt){
    traj.push_back(roomba);
    motion(roomba, v, y);
  }
  traj.push_back(roomba);

  return;
}

void log_best_traj(const std::vector<Status>& l_traj){
  geometry_msgs::PoseStamped lpath_point;
  lpath_point.pose.position.z = 0.0;

  lpath.poses.clear();
  lpath.header.frame_id = "base_link";

  for(int i = 0; i < l_traj.size(); i++){
    lpath_point.header.frame_id = "base_link";
    lpath_point.pose.position.x = l_traj[i].x;
    lpath_point.pose.position.y = l_traj[i].y;
    lpath_point.pose.orientation = tf::createQuaternionMsgFromYaw(l_traj[i].yaw);

    lpath.poses.push_back(lpath_point);
  }
  
  return;
}

double calc_to_g_goal_cost(const std::vector<Status>& l_traj, const Status& g_roomba, const Position& g_goal)
{
  Status l_last = l_traj.back();
  Position g_last = {0.0, 0.0, 0.0};
  double to_g_goal_dis = 0.0;
  double s = std::sin(g_roomba.yaw);
  double c = std::cos(g_roomba.yaw);

  g_last.x = g_roomba.x + l_last.x*c - l_last.y*s;
  g_last.y = g_roomba.y + l_last.x*s + l_last.y*c;

  to_g_goal_dis = calc_dist(g_goal.x, g_last.x, g_goal.y, g_last.y);

  return to_g_goal_cost_gain*to_g_goal_dis;
}

double calc_to_g_path_cost(const std::vector<Status>& l_traj, const Status& g_roomba, const nav_msgs::Path& g_path)
{
  Position g_goal = {0.0, 0.0, 0.0};
  Position g_path_point = {0.0, 0.0, 0.0};
  int n_g_path_p_d_i = 0;//nearest gpath point distance i
  int next_g_path_p_i = 0;
  double n_g_path_p_d = 10000.0;//nearest gpath point distance
  double g_error_dist = 0.0;
  double to_g_path_cost = 0.0;

  if(!can_goal){
	  //スタートでg_path.poses[0]が最も近い点となるように-5している
	  for(int i = 0; i < g_path.poses.size() - 5; i++){
		g_path_point.x = g_path.poses[i].pose.position.x;
		g_path_point.y = g_path.poses[i].pose.position.y;
		g_error_dist = calc_dist(g_path_point.x, g_roomba.x, g_path_point.y, g_roomba.y);

		if(n_g_path_p_d > g_error_dist){
		  n_g_path_p_d = g_error_dist;
		  n_g_path_p_d_i = i;
		}
	  }
	  next_g_path_p_i = n_g_path_p_d_i + (int)predict_time;
	  if(next_g_path_p_i >= g_path.poses.size() - 5){
		can_goal = true;
		//std::cout << "can_goal = " << can_goal << std::endl;
		next_g_path_p_i = g_path.poses.size() - 1;
	  }
  } else {
	next_g_path_p_i = g_path.poses.size() - 1;
  }

  g_goal.x = g_path.poses[next_g_path_p_i].pose.position.x;
  g_goal.y = g_path.poses[next_g_path_p_i].pose.position.y;
  g_goal.yaw = tf::getYaw(g_path.poses[next_g_path_p_i].pose.orientation);

  //publish用
  gpath_goal.header.frame_id = "map";
  gpath_goal.pose.position.x = g_goal.x;
  gpath_goal.pose.position.y = g_goal.y;
  gpath_goal.pose.position.z = 0.0;
  gpath_goal.pose.orientation = tf::createQuaternionMsgFromYaw(g_goal.yaw);

  to_g_path_cost = calc_to_g_goal_cost(l_traj, g_roomba, g_goal);

  return to_g_path_cost;
}

//全部local
double calc_l_ob_cost(const std::vector<Status>& traj, const std::vector<float>& obstacle)
{
  const int skip_i = 1;
  const int skip_j = 10;
  const double inf = std::numeric_limits<double>::infinity();
  const double left_rod_max = 1.30;
  const double left_rod_min = 0.80;
  const double right_rod_max = -0.80;
  const double right_rod_min = -1.30;
  double x = 0.0;
  double y = 0.0;
  double ob_dist = 0.0;
  double ob_theta = 0.0;
  double dist = 0.0;
  double min_dist = inf;
  double final_dist = inf;
  Position ob = {0.0, 0.0, 0.0};

  for(int i = 0; i < traj.size(); i += skip_i){
    x = traj[i].x;
    y = traj[i].y;
    ob_theta = roomba_scan.angle_min;

    for(int j = 0; j < obstacle.size(); j += skip_j){
      if(( left_rod_min < ob_theta && ob_theta < left_rod_max)
      || ( right_rod_min < ob_theta && ob_theta < right_rod_max)) {
        ob_theta += skip_j*roomba_scan.angle_increment;
        continue;
      }

      if(obstacle[j] < 60.0f) ob_dist = obstacle[j];
      else ob_dist = 60.0;

	  ob.x = ob_dist*std::cos(ob_theta);
	  ob.y = ob_dist*std::sin(ob_theta);
      dist = calc_dist(ob.x, x, ob.y, y);

      if(dist <= roomba_radius) return inf;

      if(min_dist > dist) min_dist = dist;

      ob_theta += (double)skip_j*roomba_scan.angle_increment;
    }
	final_dist = min_dist;
  }

  return l_ob_cost_gain/final_dist;
}

Speed dwa_control(const Status& g_roomba, const std::vector<float>& l_ob, const nav_msgs::Path& g_path)
{
  Dw dw = {0.0, 0.0, 0.0, 0.0};
  Speed best_output = {0.0, 0.0};
  std::vector<Status> l_traj;
  double y = 0.0;
  double ab_dwy = 0.0;
  double min_cost = 1000.0;
  double l_ob_cost = 0.0;
  double final_cost = 0.0;
  double to_g_path_cost = 0.0;
  double min_ob_cost = 0.0;
  double min_to_goal_cost = 0.0;
  double min_to_gpath_cost = 0.0;

  //dynamic windowの計算
  calc_dynamic_window(dw, g_roomba);
  for(double v = dw.min_v; v <= dw.max_v; v += dv){
    for(double dwy = dw.min_omega; dwy <= dw.max_omega; dwy += dyaw){
	  ab_dwy = fabs(dwy);
	  if(ab_dwy <= EPS) y = 0.0;
	  else y = dwy;
      //std::cout << "-------------------------------------------------------------------------------------------" << std::endl;
      //std::cout << "v = " << v << std::endl;
      //std::cout << "y = " << y << std::endl;

      //軌道計算
      calc_l_traj(l_traj, v, y);

      //cost計算
      l_ob_cost = calc_l_ob_cost(l_traj, l_ob);
      to_g_path_cost = calc_to_g_path_cost(l_traj, g_roomba, g_path);
	  
      final_cost = to_g_path_cost + l_ob_cost;

      //計算結果出力
      //std::cout << "ob_cost = " << l_ob_cost << std::endl;
      //std::cout << "to_goal_cost = " << to_g_goal_cost << std::endl;
      //std::cout << "to_gpath_cost = " << to_g_path_cost << std::endl;
      //std::cout << "final_cost = " << final_cost << std::endl;
      //std::cout << "---------------------------------------------------------------------" << std::endl;
      //std::cout << "now best_output.v = " << best_output.v << std::endl;
      //std::cout << "now best_output.omega = " << best_output.omega << std::endl;
      //std::cout << "now min ob_cost =" << min_ob_cost << std::endl;
      //std::cout << "now min to_goal_cost = " << min_to_goal_cost << std::endl;
      //std::cout << "now min to_gpath_cost = " << min_to_gpath_cost << std::endl;
      //std::cout << "now min cost =" << min_cost << std::endl;
      //std::cout << "---------------------------------------------------------------------" << std::endl;

      if(min_cost > final_cost) {
        best_output.v = v;
        best_output.omega = y;
        min_cost = final_cost;
        min_ob_cost = l_ob_cost;
        min_to_gpath_cost = to_g_path_cost;
		log_best_traj(l_traj);
      }
    }
  }

  return best_output;
}

//全部global//ゴール判別
int is_goal(const Status& roomba, const Position& goal)
{
  double error_dist = 0.0;

  error_dist = calc_dist(goal.x, roomba.x, goal.y, roomba.y);

  if(can_goal && error_dist < roomba_radius){
  	std::cout << "goal" << std::endl;
  	return 0;
  } else {
    return 11;
  }
}

bool is_normalized(const geometry_msgs::Quaternion& msg)
{
  const double quaternion_tolerance = 0.10;
  tf::Quaternion bt = tf::Quaternion(msg.x, msg.y, msg.z, msg.w);

  if(fabs(bt.length2() - 1.0) > quaternion_tolerance){
    std::cout << "quaternion unnormalized" << std::endl;
    return false;
  } else {
    return true;
  }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  roomba_odom = *msg;
  get_odom = true;
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  roomba_scan = *msg;
}

void gpath_callback(const nav_msgs::Path::ConstPtr& msg)
{
  roomba_gpath = *msg;
}

void amcl_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  roomba_status = *msg;
  get_pose = true;
}

void line_detection_callback(const std_msgs::Bool::ConstPtr& msg){
  line_detection = msg->data;
  get_line_detection = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dwa");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  ros::Publisher roomba_cntl_pub = n.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
  ros::Publisher lpath_pub = n.advertise<nav_msgs::Path>("lpath", 1);
  ros::Publisher gpath_goal_pub = n.advertise<geometry_msgs::PoseStamped>("gpath_goal", 1);
  ros::Subscriber roomba_odom_sub = n.subscribe("roomba/odometry", 1, odom_callback);
  ros::Subscriber roomba_scan_sub = n.subscribe("scan",1, scan_callback);
  ros::Subscriber roomba_gpath_sub = n.subscribe("gpath", 1, gpath_callback);
  ros::Subscriber roomba_status_sub = n.subscribe("amcl_pose", 1, amcl_callback);
  ros::Subscriber line_detection_sub = n.subscribe("detection", 1, line_detection_callback);
  ros::Rate loop_rate(5.0);

  nh.param("dt", dt, 0.0);
  nh.param("dv", dv, 0.0);
  nh.param("dyaw", dyaw, 0.0);
  nh.param("stop_time", stop_time, 0.0);
  nh.param("max_speed", max_speed, 0.0);
  nh.param("min_speed", min_speed, 0.0);
  nh.param("max_accel", max_accel, 0.0);
  nh.param("ignore_line", ignore_line, 0.0);
  nh.param("limit_speed", limit_speed, 0.0);
  nh.param("max_yawrate", max_yawrate, 0.0);
  nh.param("predict_time", predict_time, 0.0);
  nh.param("max_dyawrate", max_dyawrate, 0.0);
  nh.param("limit_yawrate", limit_yawrate, 0.0);
  nh.param("roomba_radius", roomba_radius, 0.0);
  nh.param("l_ob_cost_gain", l_ob_cost_gain, 0.0);
  nh.param("to_g_goal_cost_gain", to_g_goal_cost_gain, 0.0);

  roomba_500driver_meiji::RoombaCtrl roomba_cntl;

  Speed output = {0.0, 0.0};
  Status g_roomba = {0.0, 0.0, 0.0, 0.0, 0.0};
  Position g_goal = {0.0, 0.0, 0.0};
  Position detected_line = {0.0, 0.0, 0.0};
  bool flags = false;
  bool normalized = false;
  bool invalid_l_d = false;
  double line_dist = 0.0;
  //std::chrono::system_clock::time_point start, end;

  while (ros::ok()){
    //start = std::chrono::system_clock::now();
    normalized = is_normalized(roomba_status.pose.orientation);
    flags = !roomba_scan.ranges.empty() && !roomba_gpath.poses.empty() && normalized && get_odom && get_pose && get_line_detection;
    if(flags){
	  //goal設定
	  g_goal.x = roomba_gpath.poses.back().pose.position.x;
	  g_goal.y = roomba_gpath.poses.back().pose.position.y;

	  //現在位置設定
	  g_roomba.x = roomba_status.pose.position.x;
	  g_roomba.y = roomba_status.pose.position.y;
	  g_roomba.yaw = tf::getYaw(roomba_status.pose.orientation);
	  g_roomba.v = max_speed*roomba_odom.twist.twist.linear.x;
	  g_roomba.omega = max_yawrate*roomba_odom.twist.twist.angular.z;

	  //白線検知
      if(line_detection && !invalid_l_d){
      	roomba_cntl.mode = 0; 
        roomba_cntl.cntl.linear.x = 0.0;
        roomba_cntl.cntl.angular.z = 0.0;
	    roomba_cntl_pub.publish(roomba_cntl);
		ros::Duration(stop_time).sleep();
      	invalid_l_d = true;
      	detected_line.x = g_roomba.x;
      	detected_line.y = g_roomba.y;
      }

	  //白線を検知して止まった場所からの距離を計算
	  if(invalid_l_d){
	    line_dist = calc_dist(g_roomba.x, detected_line.x, g_roomba.y, detected_line.y);
	    if(line_dist > ignore_line) invalid_l_d = false;
	  }

	  //出力速度の計算
	  output = dwa_control(g_roomba, roomba_scan.ranges, roomba_gpath);

	  roomba_cntl.mode = is_goal(g_roomba, g_goal);
	  //roomba_cntl.mode = 0; 
	  //std::cout << "roomba_cntl.mode = " << roomba_cntl.mode << std::endl;
	  //std::cout << "roomba_cntl.cntl.linear.x = " << roomba_cntl.cntl.linear.x << std::endl;
	  //std::cout << "roomba_cntl.cntl.linear.y = " << roomba_cntl.cntl.linear.y << std::endl;
	  roomba_cntl.cntl.linear.x = output.v/max_speed;
	  roomba_cntl.cntl.angular.z = output.omega/max_yawrate;
	  roomba_cntl_pub.publish(roomba_cntl);
	  lpath_pub.publish(lpath);
	  gpath_goal_pub.publish(gpath_goal);
    }
    //end = std::chrono::system_clock::now();
    //auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    //std::cout << "duration = " << msec << " msec" << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

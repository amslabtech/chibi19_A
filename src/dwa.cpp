#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include <cmath>
#include <vector>
#include <limits>
#include <sstream>

nav_msgs::Path roomba_gpath;
nav_msgs::Path roomba_lpath;
nav_msgs::Odometry roomba_odom;
sensor_msgs::LaserScan roomba_scan;
geometry_msgs::PoseStamped gpath_goal;
geometry_msgs::PoseStamped roomba_status;

bool dwa_only;
bool get_odom = false;
bool get_pose = false;
bool get_gpath = false;
double dt;
double dv;
double dyaw;
double max_speed;
double min_speed;
double max_accel;
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

  if(!x){
    if(!y){
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

  roomba_lpath.poses.clear();
  roomba_lpath.header.frame_id = "base_link";

  for(int i = 0; i < l_traj.size(); i++){
    lpath_point.header.frame_id = "base_link";
	lpath_point.pose.position.x = l_traj[i].x;
	lpath_point.pose.position.y = l_traj[i].y;
	lpath_point.pose.orientation = tf::createQuaternionMsgFromYaw(l_traj[i].yaw);

	roomba_lpath.poses.push_back(lpath_point);
  }
  
  return;
}

double calc_to_g_goal_cost(const std::vector<Status>& l_traj, const Status& g_roomba, const Position& g_goal)
{
  Status l_last = l_traj.back();
  Position g_last = {0.0, 0.0, 0.0};
  Position g_error = {0.0, 0.0, 0.0};
  double l_last_r = 0.0;
  double l_last_theta = 0.0;
  double g_error_theta = 0.0;
  double to_g_goal_theta = 0.0;

  l_last_r = std::sqrt(l_last.x*l_last.x + l_last.y*l_last.y);
  l_last_theta = atan(l_last.x, l_last.y);

  g_last.x = g_roomba.x + l_last_r*std::cos(g_roomba.yaw + l_last_theta);
  g_last.y = g_roomba.y + l_last_r*std::sin(g_roomba.yaw + l_last_theta);
  g_last.yaw = g_roomba.yaw + l_last.yaw;
  //std::cout << "g_roomba.yaw = " << g_roomba.yaw << std::endl;
  //std::cout << "l_last.yaw = " << l_last.yaw << std::endl;
  //std::cout << "g_last.yaw = " << g_last.yaw << std::endl;

  g_error.x = g_goal.x - g_last.x;
  g_error.y = g_goal.y - g_last.y;
  g_error_theta = atan(g_error.x, g_error.y);
  //std::cout << "g_error_theta = " << g_error_theta << std::endl;

  to_g_goal_theta = g_error_theta - g_last.yaw;
  angle_range(to_g_goal_theta);
  to_g_goal_theta = std::fabs(to_g_goal_theta);
  if(to_g_goal_theta > M_PI/2) to_g_goal_theta = M_PI/2;
  to_g_goal_theta = std::sin(to_g_goal_theta);

  return to_g_goal_cost_gain*to_g_goal_theta;
}

double calc_to_g_path_cost(const std::vector<Status>& l_traj, const Status& g_roomba, const nav_msgs::Path& g_path)
{
  Position g_goal = {0.0, 0.0, 0.0};
  Position g_error = {0.0, 0.0, 0.0};
  int n_g_path_p_d_i = 0;//nearest gpath point distance i
  int next_g_path_p_i = 0;
  double n_g_path_p_d = 10000.0;//nearest gpath point distance
  double g_error_dist = 0.0;
  double to_g_path_cost = 0.0;

  for(int i = 0; i < g_path.poses.size(); i++){
    g_error.x = g_path.poses[i].pose.position.x - g_roomba.x;
    g_error.y = g_path.poses[i].pose.position.y - g_roomba.y;
    g_error_dist = std::sqrt(g_error.x*g_error.x + g_error.y*g_error.y);

    if(n_g_path_p_d > g_error_dist){
      n_g_path_p_d = g_error_dist;
      n_g_path_p_d_i = i;
    }
  }

  //gpathが環状なら
  next_g_path_p_i = (n_g_path_p_d_i + (int)predict_time) % g_path.poses.size();
  g_goal.x = g_path.poses[next_g_path_p_i].pose.position.x;
  g_goal.y = g_path.poses[next_g_path_p_i].pose.position.y;
  g_goal.yaw = tf::getYaw(g_path.poses[next_g_path_p_i].pose.orientation);

  gpath_goal.header.frame_id = "map";
  gpath_goal.pose.position.x = g_goal.x;
  gpath_goal.pose.position.y = g_goal.y;
  gpath_goal.pose.position.z = 0.0;
  gpath_goal.pose.orientation = g_path.poses[next_g_path_p_i].pose.orientation;

  to_g_path_cost = calc_to_g_goal_cost(l_traj, g_roomba, g_goal);

  return to_g_path_cost;
}

//全部local
double calc_speed_cost(const std::vector<Status>& traj)
{
  Status last = traj.back();
  double error_speed = max_speed - last.v;

  return speed_cost_gain*error_speed;
}

//全部local
double calc_l_ob_cost(const std::vector<Status>& traj, const std::vector<float>& obstacle)
{
  int skip_i = 1;
  int skip_j = 10;
  double x = 0.0;
  double y = 0.0;
  double r = 0.0;
  double ob = 0.0;
  double xx = 0.0;
  double yy = 0.0;
  double rr = 0.0;
  double rob = 0.0;
  double obob = 0.0;
  double c = 0.0;
  double theta = 0.0;
  double ob_theta = 0.0;
  double inf = std::numeric_limits<double>::infinity();
  double dist = 0.0;
  double min_dist = inf;
  double final_dist = inf;
  double left_rod_max = 1.30;
  double left_rod_min = 0.80;
  double right_rod_max = -0.80;
  double right_rod_min = -1.30;

  for(int i = 0; i < traj.size(); i += skip_i){
    x = traj[i].x;
    y = traj[i].y;
    xx = x*x;
    yy = y*y;
    r = std::sqrt(xx + yy);
    theta = atan(x, y);
    ob_theta = roomba_scan.angle_min;

    for(int j = 0; j < obstacle.size(); j += skip_j){
      if(( left_rod_min < ob_theta && ob_theta < left_rod_max)
      || ( right_rod_min < ob_theta && ob_theta < right_rod_max)) {
        ob_theta += skip_j*roomba_scan.angle_increment;
        continue;
      }

      if(obstacle[j] < 60.0f) ob = obstacle[j];
      else ob = 60.0;

      //極座標での２点間の距離を計算
      rr = r*r;
      obob = ob*ob;
      rob = r*ob;
      c = std::cos(theta - ob_theta);
      dist = std::sqrt(rr + obob - 2*rob*c);

      if(dist <= roomba_radius) return inf;

      if(min_dist > dist) min_dist = dist;

      ob_theta += (double)skip_j*roomba_scan.angle_increment;
    }
    final_dist = min_dist;
  }

  //min_dist = std::sqrt(min_dist);
  final_dist = std::sqrt(final_dist);

  //return l_ob_cost_gain/min_dist;
  return l_ob_cost_gain/final_dist;
}

Speed dwa_control(const Status& g_roomba, const std::vector<float>& l_ob, const Position& g_goal, const nav_msgs::Path& g_path)
{
  Dw dw = {0.0, 0.0, 0.0, 0.0};
  Speed best_output = {0.0, 0.0};
  std::vector<Status> l_traj;
  const double EPS = 1e-6;
  double min_cost = 1000.0;
  double l_ob_cost = 0.0;
  double final_cost = 0.0;
  double speed_cost = 0.0;
  double to_g_goal_cost = 0.0;
  double to_g_path_cost = 0.0;
  double exception_omega = max_yawrate*0.10;
  double min_ob_cost = 0.0;
  double min_speed_cost = 0.0;;
  double min_to_goal_cost = 0.0;
  double min_to_gpath_cost = 0.0;
  double y = 0.0;
  double ab_dwy = 0.0;

  //dynamic windowの計算
  calc_dynamic_window(dw, g_roomba);

  for(double v = dw.min_v; v <= dw.max_v; v += dv){
    for(double dwy = dw.min_omega; dwy <= dw.max_omega; dwy += dyaw){
	  ab_dwy = fabs(dwy);
      if(EPS < ab_dwy && ab_dwy < exception_omega) continue;
	  else if(ab_dwy <= EPS) y = 0.0;
	  else y = dwy;
      std::cout << "-------------------------------------------------------------------------------------------" << std::endl;
      std::cout << "v = " << v << std::endl;
      std::cout << "y = " << y << std::endl;

      //軌道計算
      calc_l_traj(l_traj, v, y);

      //cost計算
      l_ob_cost = calc_l_ob_cost(l_traj, l_ob);
      speed_cost = calc_speed_cost(l_traj);
      if(dwa_only) to_g_goal_cost = calc_to_g_goal_cost(l_traj, g_roomba, g_goal);
      else to_g_path_cost = calc_to_g_path_cost(l_traj, g_roomba, g_path);
      final_cost = to_g_goal_cost + to_g_path_cost + speed_cost + l_ob_cost;

      //計算結果出力
      std::cout << "ob_cost = " << l_ob_cost << std::endl;
      std::cout << "speed_cost = " << speed_cost << std::endl;
      std::cout << "to_goal_cost = " << to_g_goal_cost << std::endl;
      std::cout << "to_gpath_cost = " << to_g_path_cost << std::endl;
      std::cout << "final_cost = " << final_cost << std::endl;
      std::cout << "---------------------------------------------------------------------" << std::endl;
      std::cout << "now best_output.v = " << best_output.v << std::endl;
      std::cout << "now best_output.omega = " << best_output.omega << std::endl;
      std::cout << "now min ob_cost =" << min_ob_cost << std::endl;
      std::cout << "now min speed_cost =" << min_speed_cost << std::endl;
      std::cout << "now min to_goal_cost = " << min_to_goal_cost << std::endl;
      std::cout << "now min to_gpath_cost = " << min_to_gpath_cost << std::endl;
      std::cout << "now min cost =" << min_cost << std::endl;
      std::cout << "---------------------------------------------------------------------" << std::endl;

      if(min_cost > final_cost) {
        best_output.v = v;
        best_output.omega = y;
        min_cost = final_cost;
        min_ob_cost = l_ob_cost;
        min_speed_cost = speed_cost;
        min_to_goal_cost = to_g_goal_cost;
        min_to_gpath_cost = to_g_path_cost;
		log_best_traj(l_traj);
      }
    }
  }

  if(min_cost > 999.0) {
    best_output.v = 0.0;
    best_output.omega = 0.2;
  }

  return best_output;
}

//全部global//ゴール判別
int is_goal(const Status& roomba, const Position& goal)
{
  Position error = {0.0, 0.0, 0.0};
  error.x = goal.x - roomba.x;
  error.y = goal.y - roomba.y;
  error.yaw = goal.yaw - roomba.yaw;
  double error_dist = std::sqrt(error.x*error.x + error.y*error.y);
  double yaw_tolerance = M_PI/90;

  if(error_dist < roomba_radius && error.yaw < yaw_tolerance){
    std::cout << "goal" << std::endl;
    return 0;
  } else {
    return 11;
  }
}

bool is_normalized(const geometry_msgs::Quaternion& msg)
{
  const double quaternion_tolerance = 0.1;
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
  get_gpath = true;
}

void amcl_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  roomba_status = *msg;
  get_pose = true;
  //std::cout << "get pose from amcl" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dwa");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  ros::Publisher roomba_cntl_pub = n.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
  ros::Publisher roomba_lpath_pub = n.advertise<nav_msgs::Path>("lpath", 1);
  ros::Publisher gpath_goal_pub = n.advertise<geometry_msgs::PoseStamped>("gpath_goal", 1);
  ros::Subscriber roomba_odom_sub = n.subscribe("roomba/odometry", 1, odom_callback);
  ros::Subscriber roomba_scan_sub = n.subscribe("scan",1, scan_callback);
  ros::Subscriber roomba_gpath_sub = n.subscribe("gpath", 1, gpath_callback);
  ros::Subscriber roomba_status_sub = n.subscribe("amcl_pose", 1, amcl_callback);
  ros::Rate loop_rate(5.0);

  nh.param("dwa_only", dwa_only, false);//dwaのみ試したいときにdwa.yamlでtrueにする
  nh.param("dt", dt, 0.0);
  nh.param("dv", dv, 0.0);
  nh.param("dyaw", dyaw, 0.0);
  nh.param("max_speed", max_speed, 0.0);
  nh.param("min_speed", min_speed, 0.0);
  nh.param("max_accel", max_accel, 0.0);
  nh.param("limit_speed", limit_speed, 0.0);
  nh.param("max_yawrate", max_yawrate, 0.0);
  nh.param("max_dyawrate", max_dyawrate, 0.0);
  nh.param("limit_yawrate", limit_yawrate, 0.0);
  nh.param("predict_time", predict_time, 0.0);
  nh.param("roomba_radius", roomba_radius, 0.0);
  nh.param("l_ob_cost_gain", l_ob_cost_gain, 0.0);
  nh.param("speed_cost_gain", speed_cost_gain, 0.0);
  nh.param("to_g_goal_cost_gain", to_g_goal_cost_gain, 0.0);

  roomba_500driver_meiji::RoombaCtrl roomba_cntl;

  Speed output = {0.0, 0.0};
  Position g_goal = {3.0, 3.0, 0.0};
  Status g_roomba = {0.0, 0.0, 0.0, 0.0, 0.0};
  bool flags = false;
  bool normalized = false;

  while (ros::ok()){
    if(dwa_only){
      normalized = is_normalized(roomba_odom.pose.pose.orientation);
      flags = !roomba_scan.ranges.empty() && normalized && get_odom;
    } else {
      normalized = is_normalized(roomba_status.pose.orientation);
      flags = !roomba_scan.ranges.empty() && normalized && get_odom && get_pose && get_gpath;
    }
    if(flags){
      if(dwa_only) {
        g_roomba.x = roomba_odom.pose.pose.position.x;
        g_roomba.y = roomba_odom.pose.pose.position.y;
        g_roomba.yaw = tf::getYaw(roomba_odom.pose.pose.orientation);
        roomba_cntl.mode = is_goal(g_roomba, g_goal);
      } else {
        g_roomba.x = roomba_status.pose.position.x;
        g_roomba.y = roomba_status.pose.position.y;
        g_roomba.yaw = tf::getYaw(roomba_status.pose.orientation);
        roomba_cntl.mode = 11;//暫定
      }
      g_roomba.v = max_speed*roomba_odom.twist.twist.linear.x;
      g_roomba.omega = max_yawrate*roomba_odom.twist.twist.angular.z;

      output = dwa_control(g_roomba, roomba_scan.ranges, g_goal, roomba_gpath);

      //roomba_cntl.cntl.linear.x = 0.40/max_speed;
      //roomba_cntl.cntl.angular.z = 1.0;
      roomba_cntl.cntl.linear.x = output.v/max_speed;
      roomba_cntl.cntl.angular.z = output.omega/max_yawrate;
      roomba_cntl_pub.publish(roomba_cntl);

      roomba_lpath_pub.publish(roomba_lpath);
      gpath_goal_pub.publish(gpath_goal);
      //std::cout << "roomba_cntl.cntl.linear.x = " << roomba_cntl.cntl.linear.x << std::endl;
      //std::cout << "roomba_cntl.cntl.angular.z = " << roomba_cntl.cntl.angular.z << std::endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

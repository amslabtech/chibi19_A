#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>

bool map_received = false;

struct landmark{
	double x;
	double y;
};


struct Open{
	int f;
	int g;
	int h;
	int x;
	int y;
};

class A_star
{
private:
	nav_msgs::Path roomba_gpath;
	geometry_msgs::PoseStamped roomba_status;
	nav_msgs::OccupancyGrid map;
	std::vector<std::vector<char> > grid;
	std::vector<std::vector<int> > heuristic;
	std::vector<int> init;
	std::vector<int> goal;

	unsigned int map_row;
	unsigned int map_col;

	ros::NodeHandle nh;
	ros::Publisher roomba_gpath_pub;
	ros::Subscriber map_sub;
	ros::Subscriber roomba_status_sub;

public:
	A_star(void);
	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void amcl_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void get_heuristic(int, int);
	bool search_path(float, float, float, float);
	void pub_path(void);
};

A_star::A_star(void)
{
	roomba_gpath_pub = nh.advertise<nav_msgs::Path>("dwa", 1);
	map_sub = nh.subscribe("map", 1, &A_star::map_callback,this);
	roomba_status_sub = nh.subscribe("amcl_pose", 1, &A_star::amcl_callback, this);
	roomba_gpath.header.frame_id = "map";	
}

void A_star::amcl_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	roomba_status = *msg;
}
void A_star::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	if(map_received)
		return;
	ROS_INFO("map received");
	map = *msg;

	map_row = map.info.height;
	map_col = map.info.width;

	init.resize(2);
	goal.resize(2);

	
	grid = std::vector<std::vector<char> >(map_row, std::vector<char>(map_col, 0));
	for(int row = 0; row < map_row; row++){
		for(int col = 0; col < map_col; col++){
			grid[row][col] = map.data[ row + map_row*col];
		}
	}

	heuristic = std::vector<std::vector<int> >(map_row, std::vector<int>(map_col, 0));

	map_received = true;
}

void A_star::get_heuristic(int gx, int gy)
{
	for(int row = 0; row < map_row; row++){
		for(int col = 0; col < map_col; col++){
			heuristic[row][col] = fabs(gx - row) + fabs(gy - col);
		}
	}
}

bool A_star::search_path(float ix, float iy, float gx, float gy)
{

	init[0] = floor((ix - map.info.origin.position.x) / map.info.resolution);
	init[1] = floor((iy - map.info.origin.position.y) / map.info.resolution);
	goal[0] = floor((gx - map.info.origin.position.x) / map.info.resolution);
	goal[1] = floor((gy - map.info.origin.position.y) / map.info.resolution);

	ROS_INFO("start = (%d, %d)", init[0], init[1]);
	ROS_INFO("goal = (%d, %d)", goal[0], goal[1]);

	get_heuristic(goal[0], goal[1]);


	bool found = false;
	bool resign = false;
	int row = map.info.height;
	int col = map.info.width;
	int x = init[0];
	int y = init[1];
	int g = 0;
	int h = heuristic[x][y];
	int f = g + h;
	int x2 = 0;
	int y2 = 0;
	int g2 = 0;
	int h2 = 0;
	int f2 = 0;
	int cost = 1;
	float res = map.info.resolution;
	double origin_x = map.info.origin.position.x;
	double origin_y = map.info.origin.position.y;
	geometry_msgs::PoseStamped gpath_point;
	gpath_point.header.frame_id = "map";
	//roomba_gpath.poses.clear();


	std::vector<std::vector<double> > delta = {
		{-1,  0, M_PI},
		//{-1, -1},
		{ 0, -1, -M_PI / 2},
		//{ 1, -1},
		{ 1,  0, 0},
		//{ 1,  1},
		{ 0,  1, M_PI /2}
		//{-1,  1}
	};

	std::vector<std::vector<bool> > closed(row, std::vector<bool>(col, false));
	closed[init[0]][init[1]] = true;
	std::vector<std::vector<char> > action(row, std::vector<char>(col, -1));

	Open open_init = {f, g, h, x, y};
	std::vector<Open> open;
	open.push_back(open_init);
	Open next = {0, 0, 0, 0, 0};
	Open new_open = {0, 0, 0, 0, 0};

	while(!found && !resign){
		if(!open.size()){
			resign = true;
			ROS_INFO("\nfail\n");
			return false;
		} else {
			std::sort(open.begin(), open.end(), [](const Open x, const Open y){
				return x.f > y.f;
			});
			next = open.back();
			open.pop_back();
			x = next.x;
			y = next.y;
			g = next.g;
			f = next.f;
			if(x == goal[0] && y == goal[1]){
				found = true;
				ROS_INFO("found");
			} else {
				for(int i = 0; i < delta.size(); i++){
					x2 = x + delta[i][0];
					y2 = y + delta[i][1];
					if(x2 >= 0 && x2 < row && y2 >= 0 && y2 < col){
						if(!closed[x2][y2] && !grid[x2][y2]){
							g2 = g + cost;
							h2 = heuristic[x2][y2];
							f2 = g2 + h2;

							new_open.f = f2;
							new_open.g = g2;
							new_open.h = h2;
							new_open.x = x2;
							new_open.y = y2;
							open.push_back(new_open);

							closed[x2][y2] = true;
							action[x2][y2] = i;

						}
					}
				}
			}
		}
	}
	ROS_INFO("search completed");	
	x = goal[0];
	y = goal[1];
	std::vector<geometry_msgs::PoseStamped> tmp_poses;

	gpath_point.pose.position.x = x*res + origin_x;
	gpath_point.pose.position.y = y*res + origin_y;
	gpath_point.pose.position.z = 0;
	quaternionTFToMsg(tf::createQuaternionFromYaw(delta[action[x][y]][2]), gpath_point.pose.orientation);

	tmp_poses.push_back(gpath_point);
	while(x != init[0] || y != init[1]){
		x2 = x - delta[action[x][y]][0];
		y2 = y - delta[action[x][y]][1];
		gpath_point.pose.position.x = x2*res + origin_x;
		gpath_point.pose.position.y = y2*res + origin_y;
		quaternionTFToMsg(tf::createQuaternionFromYaw(delta[action[x][y]][2]), gpath_point.pose.orientation);
		tmp_poses.push_back(gpath_point);
	
		x = x2;
		y = y2;
	}
	std::reverse(tmp_poses.begin(), tmp_poses.end());
	roomba_gpath.poses.insert(roomba_gpath.poses.end(), tmp_poses.begin(), tmp_poses.end());
	ROS_INFO("set path");
	return true;
}

void A_star::pub_path(void)
{
	roomba_gpath_pub.publish(roomba_gpath);
	ROS_INFO("publsh path");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "a_star_s");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	ros::Rate loop_rate(10);

	const int max_count = 7;
	std::vector<landmark> landmarks(max_count);

	private_nh.param("ix", landmarks[0].x, 0.0);
	private_nh.param("iy", landmarks[0].y, 0.0);
	private_nh.param("gx1", landmarks[1].x, 14.5);
	private_nh.param("gy1", landmarks[1].y, -3.5);
	private_nh.param("gx2", landmarks[2].x, 15.0);
	private_nh.param("gy2", landmarks[2].y, 10.0);
	private_nh.param("gx3", landmarks[3].x, -1.0);
	private_nh.param("gy3", landmarks[3].y, 10.5);
	private_nh.param("gx4", landmarks[4].x, -18.0);
	private_nh.param("gy4", landmarks[4].y, 11.5);
	private_nh.param("gx5", landmarks[5].x, -18.5);
	private_nh.param("gy5", landmarks[5].y, -2.3);
	private_nh.param("gx6", landmarks[6].x, 0.0);
	private_nh.param("gy6", landmarks[6].y, 0.0);


	A_star as;
	int count = 0;
	
	while(ros::ok())
	{
		if(map_received && count < max_count-1){
			if(as.search_path(landmarks[count].x, landmarks[count].y, landmarks[count+1].x, landmarks[count+1].y)){
					as.pub_path();
					count++;
			}
		}
		if(count == max_count -1){
			as.pub_path();
		}
    	ros::spinOnce();
    	loop_rate.sleep();
  	}

  return 0;
}

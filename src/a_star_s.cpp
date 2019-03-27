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
int ix;//initx
int iy;//inity
int gx;//goalx
int gy;//goaly

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
	bool search_path(void);
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

	init[0] = floor((ix - map.info.origin.position.x) / map.info.resolution);
	init[1] = floor((iy - map.info.origin.position.y) / map.info.resolution);
	goal[0] = floor((gx - map.info.origin.position.x) / map.info.resolution);
	goal[1] = floor((gy - map.info.origin.position.y) / map.info.resolution);
	
	grid = std::vector<std::vector<char> >(map_row, std::vector<char>(map_col, 0));
	for(int row = 0; row < map_row; row++){
		for(int col = 0; col < map_col; col++){
			grid[row][col] = map.data[ row + map_row*col];
		}
	}

	heuristic = std::vector<std::vector<int> >(map_row, std::vector<int>(map_col, 0));
	for(int row = 0; row < map_row; row++){
		for(int col = 0; col < map_col; col++){
			heuristic[row][col] = fabs(goal[0] - row) + fabs(goal[1] - col);
		}
	}
	map_received = true;
}

bool A_star::search_path(void)
{

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
 	     	std::sort(open.begin(), open.end(),	[](const Open x, const Open y){
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
	gpath_point.pose.position.x = x*res + origin_x;
	gpath_point.pose.position.y = y*res + origin_y;
	gpath_point.pose.position.z = 0;
	quaternionTFToMsg(tf::createQuaternionFromYaw(delta[action[x][y]][2]), gpath_point.pose.orientation);

	roomba_gpath.poses.push_back(gpath_point);
	while(x != init[0] || y != init[1]){
		x2 = x - delta[action[x][y]][0];
		y2 = y - delta[action[x][y]][1];
		gpath_point.pose.position.x = x2*res + origin_x;
		gpath_point.pose.position.y = y2*res + origin_y;
		quaternionTFToMsg(tf::createQuaternionFromYaw(delta[action[x][y]][2]), gpath_point.pose.orientation);
		roomba_gpath.poses.push_back(gpath_point);
		
		x = x2;
		y = y2;
	}
	
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

	private_nh.param("ix", ix, 0);
	private_nh.param("iy", iy, 0);
	private_nh.param("gx", gx, 4);
	private_nh.param("gx", gy, 4);


	A_star as;
	int count = 0;
	while(ros::ok())
	{
		if(map_received){
			if(as.search_path()){
				as.pub_path();
				count++;
			}
		}

    	ros::spinOnce();
    	loop_rate.sleep();
  	}

  return 0;
}

#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>

#include<stdlib.h>
#include<time.h>
#include<math.h>
#include<queue>
#include<string.h>

typedef struct
{
	double x;
	double y;
	double theta;

} pose_vector;

class Odom_data
{
public:
	pose_vector pose;
	pose_vector delta;

};

class CellData
{
public:
	unsigned int i_f, j_f;//free cell
	unsigned int i_o, j_o;//occupied cell
	double *occ_dist;//障害物までの距離
};

class Particle
{
public:
	Particle(void);
	void init_set(double, double, double, double, double, double);
	void move(Odom_data);
	void sense(void);    
	pose_vector p_data;
	
	double w;

private:

};

int map_index(int, int);
int map_grid(double);
bool map_valid(int, int);
double normalize(double);
double angle_diff(double, double);
double gaussian(double);//Box-Muller
void enqueue(int, int, int, int, std::priority_queue<CellData>&, unsigned char*, int);
void map_update_cspace(void);
void resample(double);
void estimate_pose(void);
void filter_update(void);

nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid dist;
sensor_msgs::LaserScan laser;
geometry_msgs::PoseWithCovarianceStamped init_pose;
geometry_msgs::PoseStamped estimated_pose;
geometry_msgs::PoseArray p_poses;
double *occ_dist;

int N;
double init_x;
double init_y;
double init_theta;
double init_x_cov;
double init_y_cov;
double init_theta_cov;
double x_cov;
double y_cov;
double theta_cov;
double x_cov_thresh;
double y_cov_thresh;
double alpha_slow;
double alpha_fast;
double motion_update;
double angle_update;
double motion = 0.0;
double angle = 0.0;
double w_slow = 0.0;
double w_fast = 0.0;
int update_count = 0;
int resample_interval;

double alpha1;
double alpha2;
double alpha3;
double alpha4;

int max_beam;
double MAX_RANGE;
double MIN_RANGE;
double z_hit;
double z_rand;
double sigma_hit;
double laser_likelihood_max_dist;
int range_count = 0.0;

bool map_received = false;
bool init_set = false;
bool use_init_pose;

std::vector<Particle> p_cloud;

void LaserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
	//ROS_INFO("laser received");
	laser = *msg;
	range_count = laser.ranges.size();
	if(range_count){
		laser.range_min = std::max(laser.range_min, (float)MIN_RANGE);
		laser.range_max = std::min(laser.range_max, (float)MAX_RANGE);
		for(int i=0; i < range_count; i++){
			if(laser.ranges[i] <= laser.range_min){
				laser.ranges[i] = laser.range_max;
			}
		}
	}
}

void MapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
	if(map_received)
		return;
	
	map = *msg;

	occ_dist = (double*)malloc(sizeof(double) * map.info.width * map.info.height);
	
	if(!use_init_pose){
		for(int i=0; i < N; i++){
			Particle p;
			p.init_set(init_x, init_y, init_theta, init_x_cov, init_y_cov, init_theta_cov);
			p_cloud.push_back(p);
			geometry_msgs::Pose tmp_pose;
			tmp_pose.position.x = p.p_data.x;
			tmp_pose.position.y = p.p_data.y;
			tmp_pose.position.z = 0.0;
			quaternionTFToMsg(tf::createQuaternionFromYaw(p.p_data.theta), tmp_pose.orientation);
			p_poses.poses.push_back(tmp_pose);
		}
		init_set = true;
	}
	
	//ROS_INFO("Initializing likelihood field");
	map_update_cspace();
	//ROS_INFO("Set likelihood field");
	
/*
	dist.info.resolution = map.info.resolution;
	dist.info.width = map.info.width;
	dist.info.height = map.info.height;
	dist.info.origin = map.info.origin;
	dist.data.resize(map.info.width * map.info.height);

	for(int i=0; i< map.info.width; i++){
		for(int j=0; j<map.info.height; j++){
			if(occ_dist[map_index(i,j)] <laser_likelihood_max_dist){
				dist.data[map_index(i,j)] = 50 * occ_dist[map_index(i,j)];
			}
			else{
				dist.data[map_index(i,j)] = 100;
			}
		}
	}

*/

	map_received = true;

}

void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{

	if(init_set)
		return;
	//ROS_INFO("init callback");
	init_pose = *msg;
	
	for(int i=0; i < N; i++){
		Particle p;
		p.init_set(init_pose.pose.pose.position.x, init_pose.pose.pose.position.y, tf::getYaw(init_pose.pose.pose.orientation), init_x_cov, init_y_cov, init_theta_cov);
		p_cloud.push_back(p);
		geometry_msgs::Pose tmp_pose;
		tmp_pose.position.x = p.p_data.x;
		tmp_pose.position.y = p.p_data.y;
		tmp_pose.position.z = 0.0;
		quaternionTFToMsg(tf::createQuaternionFromYaw(p.p_data.theta), tmp_pose.orientation);
		p_poses.poses.push_back(tmp_pose);
	}

	init_set = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "localization");
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_("~");

	private_nh_.getParam("alpha1", alpha1);
	private_nh_.getParam("alpha2", alpha2);
	private_nh_.getParam("alpha3", alpha3);
	private_nh_.getParam("alpha4", alpha4);
	private_nh_.getParam("init_x", init_x);
	private_nh_.getParam("init_y", init_y);
	private_nh_.getParam("init_theta", init_theta);
	private_nh_.getParam("init_x_cov", init_x_cov);
	private_nh_.getParam("init_y_cov", init_y_cov);
	private_nh_.getParam("init_theta_cov", init_theta_cov);
	private_nh_.getParam("x_cov_thresh", x_cov_thresh);
	private_nh_.getParam("y_cov_thresh", y_cov_thresh);
	private_nh_.getParam("max_beam", max_beam);
	private_nh_.getParam("MAX_RANGE", MAX_RANGE);
	private_nh_.getParam("MIN_RANGE", MIN_RANGE);
	private_nh_.getParam("z_hit", z_hit);
	private_nh_.getParam("z_rand", z_rand);
	private_nh_.getParam("sigma_hit", sigma_hit);
	private_nh_.getParam("laser_likelihood_max_dist", laser_likelihood_max_dist);
	private_nh_.getParam("alpha_fast", alpha_fast);
	private_nh_.getParam("alpha_slow", alpha_slow);
	private_nh_.getParam("N", N);
	private_nh_.getParam("motion_update", motion_update);
	private_nh_.getParam("angle_update", angle_update);
	private_nh_.getParam("use_init_pose", use_init_pose);

	srand((unsigned int)time(NULL));

	x_cov = init_x_cov;
	y_cov = init_y_cov;
	theta_cov = init_theta_cov;

	p_poses.header.frame_id = "map";
	estimated_pose.header.frame_id = "map";
	dist.header.frame_id = "map";
	dist.header.stamp = ros::Time::now();

	estimated_pose.pose.position.x = init_x;
	estimated_pose.pose.position.y = init_y;
	estimated_pose.pose.position.z = 0.0;
	quaternionTFToMsg(tf::createQuaternionFromYaw(init_theta), estimated_pose.pose.orientation);	
	ros::Publisher pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("amcl_pose", 10);
	ros::Publisher poses_pub = nh_.advertise<geometry_msgs::PoseArray>("particle", 10);
	ros::Publisher dist_pub = nh_.advertise<nav_msgs::OccupancyGrid>("likelihood", 10);
	ros::Subscriber laser_sub = nh_.subscribe("scan", 10, LaserCallback);
	ros::Subscriber map_sub = nh_.subscribe("map", 10, MapCallback);
	ros::Subscriber init_sub = nh_.subscribe("initialpose", 10, InitPoseCallback);
	
	tf::TransformListener listener;
	tf::TransformBroadcaster map_br;
	tf::StampedTransform latest_transform;
	tf::StampedTransform previous_transform;
	tf::Quaternion q(0, 0, 0, 1);
	tf::Transform transform;
	transform.setRotation(q);
	transform.setOrigin(tf::Vector3(0, 0, 0));
	previous_transform = tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link");
	
	ros::Rate loop_rate(10.0);
	while(ros::ok())
	{   
		if(map_received && range_count && init_set){
			Odom_data odom; 
			try{
				ros::Time now = ros::Time::now();
				listener.waitForTransform("odom", "base_link",now, ros::Duration(1.0));
				listener.lookupTransform("odom", "base_link",now,latest_transform);
			}
			catch(tf::TransformException &ex){
				ROS_ERROR("%s", ex.what());
			}
			odom.pose.x = latest_transform.getOrigin().x();
			odom.pose.y = latest_transform.getOrigin().y();
			odom.pose.theta = tf::getYaw(latest_transform.getRotation());
			odom.delta.x = latest_transform.getOrigin().x() - previous_transform.getOrigin().x();
			odom.delta.y= latest_transform.getOrigin().y() - previous_transform.getOrigin().y();
			odom.delta.theta = tf::getYaw(latest_transform.getRotation()) - tf::getYaw(previous_transform.getRotation());

			motion += sqrt((odom.delta.x * odom.delta.x) + (odom.delta.y * odom.delta.y));
			angle += fabs(odom.delta.theta);

			previous_transform = latest_transform;
			//ROS_INFO("x: %f, y: %f, theta: %f", odom.delta.x, odom.delta.y, odom.delta.theta);
			
			if(x_cov < x_cov_thresh && y_cov < y_cov_thresh){
				filter_update();
				//ROS_INFO("filter update");
			}
			//ROS_INFO("x_cov = %f, y_cov = %f", x_cov, y_cov);
	
			double total_w = 0.0;

			for(int i=0; i < N; i++){
				p_cloud[i].move(odom);
				p_cloud[i].sense();

				int mi = map_grid(p_cloud[i].p_data.x);
				int mj = map_grid(p_cloud[i].p_data.y);
				if((map.data[map_index(mi, mj)] == -1) || (map.data[map_index(mi, mj)] == 100)){
					p_cloud[i].w = 0.0;
				}
				//ROS_INFO("w = %f", p_cloud[i].w);
				total_w += p_cloud[i].w; 
			}
			if(motion > motion_update){
				resample(total_w);
				//ROS_INFO("resampling");
				motion = 0.0;
			}
			if(angle > angle_update){
				resample(total_w);
				//ROS_INFO("resampling");
				angle = 0.0;
			}
			estimate_pose();
			estimated_pose.header.stamp = laser.header.stamp;
			pose_pub.publish(estimated_pose);
			//ROS_INFO("published estimated_pose");
			p_poses.poses.clear();
			for(int i=0; i < N; i++){
				geometry_msgs::Pose tmp_pose;
				tmp_pose.position.x = p_cloud[i].p_data.x;
				tmp_pose.position.y = p_cloud[i].p_data.y;
				tmp_pose.position.z = 0.0;
				quaternionTFToMsg(tf::createQuaternionFromYaw(p_cloud[i].p_data.theta), tmp_pose.orientation);
				p_poses.poses.push_back(tmp_pose);

			}
			poses_pub.publish(p_poses);
		//	dist_pub.publish(dist);
			try{
				tf::Transform map_to_base;
				quaternionMsgToTF(estimated_pose.pose.orientation, q);
				map_to_base.setRotation(q);
				map_to_base.setOrigin(tf::Vector3(estimated_pose.pose.position.x, estimated_pose.pose.position.y, 0));
				
				geometry_msgs::PoseStamped base_to_map_;
				geometry_msgs::PoseStamped odom_to_map_;

				base_to_map_.header.frame_id ="base_link";
				base_to_map_.header.stamp = laser.header.stamp;
				poseTFToMsg(map_to_base.inverse(), base_to_map_.pose);
				listener.transformPose("odom", base_to_map_, odom_to_map_);
				
				tf::Transform odom_to_map;
				quaternionMsgToTF(odom_to_map_.pose.orientation, q);
				odom_to_map.setRotation(q);
				odom_to_map.setOrigin(tf::Vector3(odom_to_map_.pose.position.x, odom_to_map_.pose.position.y, 0));
				
				tf::StampedTransform map_to_odom = tf::StampedTransform(odom_to_map.inverse(), laser.header.stamp, "map", "odom");
				
				map_br.sendTransform(map_to_odom);
			}
			catch(tf::TransformException &ex){
				ROS_ERROR("%s", ex.what());
			}


		}
		ros::spinOnce();
		loop_rate.sleep();
		
	}

	free(occ_dist);
	return 0;
}

int map_index(int i, int j)
{
	return i + (map.info.width * j);
}

int map_grid(double x)
{
	return floor((x - map.info.origin.position.x) / map.info.resolution + 0.5);
}
bool map_valid(int i, int j)
{
	return((i >= 0) && (i < map.info.width) && (j >= 0) && (j < map.info.height));
}

double normalize(double z)
{
	return atan2(sin(z), cos(z));
}

double angle_diff(double a, double b)
{
	double d1, d2;
	a = normalize(a);
	b = normalize(b);
	d1 = a - b;
	d2 = 2 * M_PI - fabs(d1);
	if(d1 > 0)
		d2 *= -1.0;
	if(fabs(d1) < fabs(d2))
		return d1;
	else
		return d2;
}

double gaussian(double sigma)//Box-Muller
{
	double x1, x2, w, r;
	do{
		do{
			r = drand48();
		}while(r == 0.0);
		x1 = 2.0 * r -1.0;
		do{
			r = drand48();
		}while(r == 0.0);
		x2 = 2.0 * r -1.0;
		w = x1*x1 + x2*x2;
	}while(w > 1.0 || w==0.0);

	return (sigma * x2 * sqrt(-2.0*log(w)/w));
}

bool operator<(const CellData& a, const CellData& b)//ソートの定義
{
	return a.occ_dist[map_index(a.i_f, a.j_f)] > b.occ_dist[map_index(b.i_f, b.j_f)];
}

void enqueue(int i_f, int j_f, int i_o, int j_o, std::priority_queue<CellData>& Q, unsigned char* marked, int cell_radius)
{


	if(marked[map_index(i_f, j_f)])//同じセルかどうかチェック
		return;

	int di = abs(i_f - i_o);//freeのcellとoccupiedのcellの距離(x)
	int dj = abs(j_f - j_o);//freeのcellとoccupiedのcellの距離(y)
	double distance = sqrt((di * di) + (dj * dj));
	
	if(distance > cell_radius)
		return;

	occ_dist[map_index(i_f, j_f)] = distance * map.info.resolution;

	CellData cell;//更新
	cell.i_f = i_f;
	cell.j_f = j_f;
	cell.i_o = i_o;
	cell.j_o = j_o;
	cell.occ_dist = occ_dist;

	Q.push(cell);

	marked[map_index(i_f, j_f)] = 1;
	
}

void map_update_cspace(void)
{
	unsigned char* marked;//データを取得したcellをmarkする
	std::priority_queue<CellData> Q;//cellのデータを管理する
	marked = new unsigned char[map.info.width*map.info.height];
	memset(marked, 0, sizeof(unsigned char) * map.info.width*map.info.height);//0で初期化
	int cell_radius = laser_likelihood_max_dist / map.info.resolution;//半径
	

	CellData cell;
	cell.occ_dist = occ_dist;
	for(int i=0; i < map.info.width; i++){
		cell.i_o = i;
		cell.i_f = i;
		for(int j=0; j < map.info.height; j++){
			
			if(map.data[map_index(i, j)] == 100){// = occupied
				occ_dist[map_index(i, j)] = 0.0;
				cell.j_o = j;
				cell.j_f = j;
				marked[map_index(i, j)] = 1;//障害物があったらmark
				Q.push(cell);//marked = 1のcellをqに入れる
			}
			else{
				occ_dist[map_index(i, j)] = laser_likelihood_max_dist;
			}
		}
	}
	
	while(!Q.empty()){//freeのcellのocc_distを計算する
		CellData current_cell = Q.top();//Qのtopの要素にアクセスする

		if(current_cell.i_f > 0)
      		enqueue(current_cell.i_f-1, current_cell.j_f,//i_f-1, j_f, i_o, j_o
          		current_cell.i_o, current_cell.j_o, Q, marked, cell_radius);
    	if(current_cell.j_f > 0)
      		enqueue(current_cell.i_f, current_cell.j_f-1,//i_f, j_f-1, i_o, j_o
          		current_cell.i_o, current_cell.j_o, Q, marked, cell_radius);
    	if((int)current_cell.i_f < map.info.width - 1)
      		enqueue(current_cell.i_f+1, current_cell.j_f,//i_f+1, j_f, i_o, j_o 
          		current_cell.i_o, current_cell.j_o, Q, marked, cell_radius);
   	 	if((int)current_cell.j_f < map.info.height - 1)
      		enqueue(current_cell.i_f, current_cell.j_f+1,//i_f, j_f+1, i_o, j_o
          		current_cell.i_o, current_cell.j_o, Q, marked, cell_radius);
	
	Q.pop();//Qのtopの要素を消す
	}

  delete[] marked;

}

Particle::Particle(void)
{
	p_data.x = 0.0;
	p_data.y = 0.0;
	p_data.theta = 0.0;
	w = 1.0 / (double)N;
}

void Particle::init_set(double x, double y, double theta, double x_cov, double y_cov, double theta_cov)
{	
	double i,j;
	do{
		p_data.x = x + gaussian(x_cov);
		p_data.y = y + gaussian(y_cov);
		p_data.theta = theta + gaussian(theta_cov);
		i = map_grid(p_data.x);
		j = map_grid(p_data.y);
	}while(map.data[map_index(i, j)] != 0);
}

void Particle::move(Odom_data ndata)
{
	double delta_rot1, delta_trans, delta_rot2;
	double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
	double delta_rot1_noise, delta_rot2_noise;
	pose_vector old_pose = {(ndata.pose.x - ndata.delta.x), (ndata.pose.y - ndata.delta.y), (ndata.pose.theta - ndata.delta.theta)};

	delta_trans = sqrt((ndata.delta.x * ndata.delta.x) + (ndata.delta.y * ndata.delta.y));
	if(delta_trans < 0.01)
		delta_rot1 = 0.0;
	else
		delta_rot1 = angle_diff(atan2(ndata.delta.y, ndata.delta.x),old_pose.theta);
	
	delta_rot2 = angle_diff(ndata.delta.theta, delta_rot1);

	delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1, 0.0)), fabs(angle_diff(delta_rot1,0.0)));
	delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2, 0.0)), fabs(angle_diff(delta_rot2, M_PI)));

	delta_rot1_hat = angle_diff(delta_rot1, gaussian(alpha1*(delta_rot1_noise * delta_rot1_noise) + alpha2*(delta_trans * delta_trans)));
	delta_trans_hat = delta_trans - gaussian(alpha3*(delta_trans * delta_trans) + alpha4*(delta_rot1_noise * delta_rot1_noise) + alpha4*(delta_rot2_noise * delta_rot2_noise));
	delta_rot2_hat = angle_diff(delta_rot2, gaussian(alpha1*(delta_rot2_noise * delta_rot2_noise) + alpha2*(delta_trans * delta_trans)));

	p_data.x += delta_trans_hat * cos(p_data.theta + delta_rot1_hat);
	p_data.y += delta_trans_hat * sin(p_data.theta + delta_rot1_hat);
	p_data.theta += delta_rot1_hat + delta_rot2_hat;
	
}

void Particle::sense(void)
{

	int step;
	double z, pz;
	double p;
	double obs_range, obs_bearing;
	pose_vector hit;

	p = 1.0;

	double z_hit_demon = 2 * (sigma_hit * sigma_hit);
	double z_rand_mult = 1.0 / laser.range_max;

	step = (range_count -1) / (max_beam -1);

	if(step < 1)
		step = 1;

	for(int j=0; j<range_count; j+=step){
		obs_range = laser.ranges[j];
		obs_bearing = laser.angle_min + (laser.angle_increment * j);


		if(obs_range >= laser.range_max)
			continue;
		if(obs_range != obs_range)//check for NaN
			continue;

		pz = 0.0;

		hit.x = p_data.x + obs_range * cos(p_data.theta + obs_bearing);//laserの端点
		hit.y = p_data.y + obs_range * sin(p_data.theta + obs_bearing);

		int mi = map_grid(hit.x);
		int mj = map_grid(hit.y);
		

		if(!map_valid(mi, mj))//一番近い障害物までの距離を取得
			z = laser_likelihood_max_dist;
		else
			z = occ_dist[map_index(mi, mj)];
		
		//ROS_INFO("z = %f", z);
		pz += z_hit * exp(-(z * z) / z_hit_demon);
		pz += z_rand * z_rand_mult;

		p += pow(pz, 3.0);
	}
	w = p;
}

void resample(double total_w)
{	
	std::vector<Particle> new_p_cloud;
	new_p_cloud.resize(0);
	double mw = 0.0;
	double w_diff;
	if(total_w > 0.0){
		double w_avg = 0.0;
		for(int i=0; i < N; i++){
			w_avg += p_cloud[i].w;
			p_cloud[i].w /= total_w;
			mw = std::max(mw, p_cloud[i].w);
		}

		w_avg /= N;
		if(w_slow == 0.0)
			w_slow = w_avg;
		else
			w_slow += alpha_slow * (w_avg - w_slow);

		if(w_fast == 0.0)
			w_fast = w_avg;
		else
			w_fast += alpha_fast * (w_avg - w_fast);
	}
	else{
		for(int i=0; i < N; i++){
			p_cloud[i].w = 1.0 / double(N);
		}
	}

	w_diff = 1.0 - (w_fast / w_slow);
	if(w_diff < 0.0)
		w_diff = 0.0;
	
	int index = drand48() * N;
	double beta = 0.0;
	while(new_p_cloud.size() < N){

		if(drand48() < w_diff){
			Particle p;
			p.init_set(estimated_pose.pose.position.x, estimated_pose.pose.position.y, tf::getYaw(estimated_pose.pose.orientation), init_x_cov, init_y_cov, init_theta_cov);
			new_p_cloud.push_back(p);
		}
		else{
			beta +=	drand48() * 2.0 * mw;
			while(beta > p_cloud[index].w){
				beta -= p_cloud[index].w;
				index = (index + 1) % N;
			}
			new_p_cloud.push_back(p_cloud[index]);
		}
	}
	p_cloud = new_p_cloud;

}

void estimate_pose(void)
{
	x_cov = 0.0;
	y_cov = 0.0;
	theta_cov = 0.0;
	int count =0;
	double avg_x = 0.0;
	double avg_y = 0.0;
	double avg_theta = 0.0;
	double est_x = 0.0;
	double est_y = 0.0;
	double est_theta = 0.0;

	for(int i=0; i < N; i++){
		avg_x += p_cloud[i].p_data.x;
		avg_y += p_cloud[i].p_data.y;
		avg_theta += p_cloud[i].p_data.theta;

		if((1.0 / N) < p_cloud[i].w){
			est_x += p_cloud[i].p_data.x;
			est_y += p_cloud[i].p_data.y;
			est_theta += p_cloud[i].p_data.theta;
			count++;
		}
	}

	avg_x /= N;
	avg_y /= N;
	avg_theta /= N;

	est_x /= count;
	est_y /= count;
	est_theta /= count;

	estimated_pose.pose.position.x = est_x;
	estimated_pose.pose.position.y = est_y;
	quaternionTFToMsg(tf::createQuaternionFromYaw(est_theta), estimated_pose.pose.orientation);

	for(int i=0; i < N; i++){
		x_cov += (p_cloud[i].p_data.x - avg_x) * (p_cloud[i].p_data.x - avg_x);
		y_cov += (p_cloud[i].p_data.y - avg_y) * (p_cloud[i].p_data.y - avg_y);
		theta_cov += (p_cloud[i].p_data.theta - avg_theta) * (p_cloud[i].p_data.theta - avg_theta);
	}

	x_cov = sqrt(x_cov / N);
	y_cov = sqrt(y_cov / N);
	theta_cov = sqrt(theta_cov / N);

}

void filter_update(void)
{
	
	std::vector<Particle> new_p_cloud;
	new_p_cloud.resize(0);
	for(int i=0; i < N; i++){
		Particle p;
		p.init_set(estimated_pose.pose.position.x, estimated_pose.pose.position.y, tf::getYaw(estimated_pose.pose.orientation), init_x_cov, init_y_cov, init_theta_cov);
		new_p_cloud.push_back(p);
	}
	
	p_cloud = new_p_cloud;
}






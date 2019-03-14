#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PoseStamped.h>
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
bool map_valid(int, int);
double normalize(double);
double angle_diff(double, double);
double gaussian(double);//Box-Muller
void enqueue(int, int, int, int, std::priority_queue<CellData>&, unsigned char*, int);
void map_update_cspace(void);
void resample(double);
void estimate_pose(void);

nav_msgs::OccupancyGrid map;
sensor_msgs::LaserScan laser;
geometry_msgs::PoseStamped estimated_pose;
double *occ_dist;

double init_x;
double init_y;
double init_theta;
double init_x_cov;
double init_y_cov;
double init_theta_cov;
double cov_x;
double cov_y;
double cov_theta;
double alpha_slow;
double alpha_fast;
double w_slow;
double w_fast;

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
int range_count;
const int N = 5000;

bool map_received = false;

std::vector<Particle> p_cloud;
std::vector<std::pair<int, int>> free_indices;

void laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
	laser = *msg;
	
	range_count = laser.ranges.size();
	double range_min;
	double range_max;

	range_min = std::max(laser.range_min, (float)MIN_RANGE);
	range_max = std::min(laser.range_max, (float)MAX_RANGE);
	laser.angle_increment = fmod(laser.angle_increment + 5*M_PI, 2*M_PI) - M_PI;
	for(int i=0; i < range_count; i++){
		if(laser.ranges[i] <= range_min){
			laser.ranges[i] = range_max;
		}
	}
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
	if(map_received){
		return;
	}
	
	map = *msg;

/*	free_indices.resize(0);
	for(int i=0; i < map.info.width; i++){
		for(int j=0; j < map.info.height; j++){
			int index = i + j*map.info.width;
			if(map.data[index] == 0){
				free_indices.push_back(std::make_pair(i, j));
			}
		}
	}
*/	
	occ_dist = (double*)malloc(sizeof(double) * map.info.width * map.info.height);
	
	for(int i=0; i < N; i++){
		Particle p;
		p.init_set(init_x, init_y, init_theta, init_x_cov, init_y_cov, init_theta_cov);
		p_cloud.push_back(p);
	}
	
	ROS_INFO("Initializing likelihood field");
	map_update_cspace();
				
	map_received = true;

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "amcll");
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_("~");

	private_nh_.param("alpha1", alpha1, 0.2);
	private_nh_.param("alpha2", alpha2, 0.2);
	private_nh_.param("alpha3", alpha3, 0.2);
	private_nh_.param("alpha4", alpha4, 0.2);
	private_nh_.param("init_x", init_x, 0.0);
	private_nh_.param("init_y", init_y, 0.0);
	private_nh_.param("init_theta", init_theta, 0.0);
	private_nh_.param("init_x_cov", init_x_cov, 0.5*0.5);
	private_nh_.param("init_y_cov", init_y_cov, 0.5*0.5);
	private_nh_.param("init_theta_cov", init_theta_cov, M_PI/12.0 * M_PI/12.0);
	private_nh_.param("max_beam", max_beam, 30);
	private_nh_.param("MAX_RANGE", MAX_RANGE, 30.0);
	private_nh_.param("MIN_RANGE", MIN_RANGE, 0.01);
	private_nh_.param("z_hit", z_hit, 0.95);
	private_nh_.param("z_rand", z_rand, 0.05);
	private_nh_.param("sigma_hit", sigma_hit, 0.2);
	private_nh_.param("laser_likelihood_max_dist", laser_likelihood_max_dist, 2.0);
	private_nh_.param("alpha_fast", alpha_fast, 0.1);
	private_nh_.param("alpha_slow", alpha_slow, 0.001);

	srand((unsigned int)time(NULL));
	range_count = 0;
	cov_x = init_x_cov;
	cov_y = init_y_cov;
	cov_theta = init_theta_cov;
	estimated_pose.header.frame_id = "map";
	estimated_pose.pose.position.x = init_x;
	estimated_pose.pose.position.y = init_y;
	estimated_pose.pose.position.z = 0.0;
	quaternionTFToMsg(tf::createQuaternionFromYaw(init_theta), estimated_pose.pose.orientation);	
	ros::Publisher pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("amcl_pose", 10);
	ros::Subscriber laser_sub = nh_.subscribe("scan", 10, laserCallback);
	ros::Subscriber map_sub = nh_.subscribe("map", 10, mapCallback);
	
	tf::TransformListener listener;
	tf::StampedTransform latest_transform;
	tf::StampedTransform previous_transform;
	tf::Transform diff_transform;
	tf::Quaternion q;
	tf::Transform transform;
	q.setRPY(0.0, 0.0, 0.0);
	transform.setRotation(q);
	transform.setOrigin(tf::Vector3(0, 0, 0));
	previous_transform = tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link");
	ros::Rate loop_rate(10.0);

	while(ros::ok())
	{   
		if(map_received && range_count){
			Odom_data odom; 
			try{
				ros::Time now = ros::Time::now();
				listener.waitForTransform("odom", "base_link",now, ros::Duration(1.0));
				listener.lookupTransform("odom", "base_link",now,latest_transform);
			}
			catch(tf::TransformException &ex){
				ROS_ERROR("%s", ex.what());
			}
			diff_transform = previous_transform.inverse() * latest_transform;
			
			odom.pose.x = latest_transform.getOrigin().x();
			odom.pose.y = latest_transform.getOrigin().y();
			odom.pose.theta = tf::getYaw(latest_transform.getRotation());
			odom.delta.x = diff_transform.getOrigin().x();
			odom.delta.y= diff_transform.getOrigin().y();
			odom.delta.theta = tf::getYaw(diff_transform.getRotation()); 
	
			previous_transform = latest_transform;
			
			double total_w = 0.0;
			for(int i=0; i < N; i++){
				p_cloud[i].move(odom);
				p_cloud[i].sense();
				total_w += p_cloud[i].w; 
			}
			resample(total_w);
			estimate_pose();
			pose_pub.publish(estimated_pose);
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
	double distance = sqrt(pow(di, 2.0) + pow(dj, 2.0));

	if(distance > cell_radius)
		return;

	occ_dist[map_index(i_f, j_f)] = distance * map.info.resolution;

	CellData cell;//更新
	cell.i_f = i_f;
	cell.j_f = j_f;
	cell.i_o = i_o;
	cell.j_o = j_o;

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

/*
void Particle::init_set(void)
{
	unsigned int rand_index = drand48() * free_indices.size();
	//unsigend int rand_index = (rand() / RAND_MAX) * free_indices.size();
	std::pair<int, int> free_point = free_indices[rand_index];

	p_data.x = free_point.first * map.info.resolution;
	p_data.y = free_point.second * map.info.resolution;
	p_data.theta = drand48() * (2* M_PI) - M_PI;

}
*/
void Particle::init_set(double x, double y, double theta, double x_cov, double y_cov, double theta_cov)
{	
	double i,j;
	do{
		p_data.x = x + gaussian(x_cov);
		p_data.y = y + gaussian(y_cov);
		p_data.theta = theta + gaussian(theta_cov);
		i = floor(p_data.x / map.info.resolution + 0.5);
		j = floor(p_data.y / map.info.resolution + 0.5);
	}while(map.data[map_index(i, j)] != 0);
}

void Particle::move(Odom_data ndata)
{
	double delta_rot1, delta_trans, delta_rot2;
	double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
	double delta_rot1_noise, delta_rot2_noise;
	pose_vector old_pose = {(ndata.pose.x - ndata.delta.x), (ndata.pose.y - ndata.delta.y), (ndata.pose.theta - ndata.delta.theta)};

	delta_trans = sqrt(pow(ndata.delta.x, 2.0) + pow(ndata.delta.y, 2.0));
	if(delta_trans < 0.01)
		delta_rot1 = 0.0;
	else
		delta_rot1 = angle_diff(atan2(ndata.delta.y, ndata.delta.x),old_pose.theta);
	
	delta_rot2 = angle_diff(ndata.delta.theta, delta_rot1);

	delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1, 0.0)), fabs(angle_diff(delta_rot1,0.0)));
	delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2, 0.0)), fabs(angle_diff(delta_rot2, M_PI)));

	delta_rot1_hat = angle_diff(delta_rot1, gaussian(alpha1*pow(delta_rot1_noise,2.0) + alpha2*pow(delta_trans, 2.0)));
	delta_trans_hat = delta_trans - gaussian(alpha3*pow(delta_trans, 2.0) + alpha4*pow(delta_rot1_noise, 2.0) + alpha4*pow(delta_rot2_noise, 2.0));
	delta_rot2_hat = angle_diff(delta_rot2, gaussian(alpha1*pow(delta_rot2_noise, 2.0) + alpha2*pow(delta_trans, 2.0)));

	p_data.x += delta_trans_hat * cos(p_data.theta + delta_rot1_hat);
	p_data.y += delta_trans_hat * sin(p_data.theta + delta_rot1_hat);
	p_data.theta += delta_rot1_hat + delta_rot2_hat;
	
}

void Particle::sense(void)
{
	int range_count = laser.ranges.size();
	double range_min;
	double range_max;

	range_min = std::max(laser.range_min, (float)MIN_RANGE);
	range_max = std::min(laser.range_max, (float)MAX_RANGE);
	laser.angle_increment = fmod(laser.angle_increment + 5*M_PI, 2*M_PI) - M_PI;
	for(int i=0; i < range_count; i++){
		if(laser.ranges[i] <= range_min){
			laser.ranges[i] = range_max;
		}
	}
	int step;
	double z, pz;
	double p;
	double obs_range, obs_bearing;
	double total_weight = 0.0;
	pose_vector hit;

	p = 1.0;

	double z_hit_demon = 2 * pow(sigma_hit, 2.0);
	double z_rand_mult = 1.0 / laser.range_max;

	step = (range_count -1) / (max_beam -1);

	if(step < 1)
		step = 1;

	for(int j=0; j<range_count; j+=step){
		obs_range = laser.ranges[j];
		obs_bearing = laser.angle_min + (laser.angle_increment * j);

		if(obs_range >= range_max)
			continue;
		if(obs_range != obs_range)//check for NaN
			continue;

		pz = 0.0;
		
		hit.x = p_data.x + obs_range * cos(p_data.theta + obs_bearing);//laserの端点
		hit.y = p_data.y + obs_range * sin(p_data.theta + obs_bearing);

		int mi = floor(hit.x / map.info.resolution + 0.5);
		int mj = floor(hit.y / map.info.resolution + 0.5);

		if(!map_valid(mi, mj))//一番近い障害物までの距離を取得
			z = laser_likelihood_max_dist;
		else
			z = occ_dist[map_index(mi, mj)];

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
	int beta = 0;
	while(new_p_cloud.size() < N){

		if(drand48() < w_diff){
			Particle p;
			p.init_set(estimated_pose.pose.position.x, estimated_pose.pose.position.y, tf::getYaw(estimated_pose.pose.orientation), cov_x, cov_y, cov_theta);
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

void estimate_pose()
{
	cov_x = 0.0;
	cov_y = 0.0;
	cov_theta = 0.0;
	double avg_x = 0.0;
	double avg_y = 0.0;
	double avg_theta = 0.0;

	for(int i=0; i < N; i++){
		avg_x += p_cloud[i].p_data.x;
		avg_y += p_cloud[i].p_data.y;
		avg_theta += p_cloud[i].p_data.theta;
	}
	avg_x /= N;
	avg_y /= N;
	avg_theta /= N;
	
	estimated_pose.pose.position.x = avg_x;
	estimated_pose.pose.position.y = avg_y;
	quaternionTFToMsg(tf::createQuaternionFromYaw(avg_theta), estimated_pose.pose.orientation);

	for(int i=0; i < N; i++){
		cov_x += pow( (p_cloud[i].p_data.x - avg_x), 2.0);
		cov_y += pow( (p_cloud[i].p_data.y - avg_y), 2.0);
		cov_theta += pow( (p_cloud[i].p_data.theta - avg_theta), 2.0);
	}
	cov_x = sqrt(cov_x / N);
	cov_y = sqrt(cov_y / N);
	cov_theta = sqrt(cov_theta / N);

}








#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>

#include<stdlib.h>
#include<time.h>
#include<math.h>


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

class Particle
{
public:
	Particle(void);
	void init_set(void);
	void move(Odom_data);
    
	pose_vector p_data;
	
	double w;

private:

};


double normalize(double);
double angle_diff(double, double);
double gaussian(double);//Box-Muller

nav_msgs::OccupancyGrid map;
sensor_msgs::LaserScan laser;

double alpha1;
double alpha2;
double alpha3;
double alpha4;
const int N = 5000;


bool map_received = false;

static std::vector<Particle> p_cloud;
static std::vector<std::pair<int, int>> free_indices;

void laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
	laser = *msg;

}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
	if(map_received){
		return;
	}
	
	map = *msg;

	free_indices.resize(0);
	for(int i=0; i < map.info.width; i++){
		for(int j=0; j < map.info.height; j++){
			int index = i + j*map.info.width;
			if(map.data[index] == 0){
				free_indices.push_back(std::make_pair(i, j));
			}
		}
	}
	
	for(int i=0; i < N; i++){
		Particle p;
		p.init_set();
		p_cloud.push_back(p);
	}
				
	map_received = true;

}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "amcl");
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_("~");

	srand((unsigned int)time(NULL));

	private_nh_.param("alpha1", alpha1, 0.2);
	private_nh_.param("alpha2", alpha2, 0.2);
	private_nh_.param("alpha3", alpha3, 0.2);
	private_nh_.param("alpha4", alpha4, 0.2);


	ros::Publisher pose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10);
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
	ros::Rate rate(10.0);

	while(ros::ok())
	{   
		if(map_received){
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
	
			for(int i=0; i < N; i++){
				p_cloud[i].move(odom);
			}

		}


	}


	return 0;
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

Particle::Particle(void)
{
	p_data.x = 0.0;
	p_data.y = 0.0;
	p_data.theta = 0.0;
	w = 1.0 / (double)N;
}


void Particle::init_set(void)
{
	unsigned int rand_index = drand48() * free_indices.size();
	//unsigend int rand_index = (rand() / RAND_MAX) * free_indices.size();
	std::pair<int, int> free_point = free_indices[rand_index];

	p_data.x = free_point.first * map.info.resolution;
	p_data.y = free_point.second * map.info.resolution;
	p_data.theta = drand48() * (2* M_PI) - M_PI;

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

	for(int i=0; i < N; i++){
		delta_rot1_hat = angle_diff(delta_rot1, gaussian(alpha1*pow(delta_rot1_noise,2.0) + alpha2*pow(delta_trans, 2.0)));
		delta_trans_hat = delta_trans - gaussian(alpha3*pow(delta_trans, 2.0) + alpha4*pow(delta_rot1_noise, 2.0) + alpha4*pow(delta_rot2_noise, 2.0));
		delta_rot2_hat = angle_diff(delta_rot2, gaussian(alpha1*pow(delta_rot2_noise, 2.0) + alpha2*pow(delta_trans, 2.0)));

	
		p_data.x += delta_trans_hat * cos(p_data.theta + delta_rot1_hat);
		p_data.y += delta_trans_hat * sin(p_data.theta + delta_rot1_hat);
		p_data.theta += delta_rot1_hat + delta_rot2_hat;
	}
}














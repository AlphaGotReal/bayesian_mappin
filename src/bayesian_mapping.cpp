#include <iostream>
#include <cmath>
#include <vector>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace random {

	double random(int32_t resolution = 1000) {
		return (rand() % resolution) / (double) resolution;  
	}

	double standard_normal() {
		double u = random();
		double v = random();
		return std::sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
	}

	double normal(double mu, double std) {
		return standard_normal() * std + mu;
	}

}

class BayesianMapping {
public:
	
	BayesianMapping(int32_t argc, char **argv) {
		
		ros::init(argc, argv, "bayesian_mapping");
		ros::NodeHandle node;

		ros::param::get("/topic/map", this->map_topic);
		ros::param::get("/topic/odom", this->odom_topic);
		ros::param::get("/topic/equation/ground", this->ground_plane_topic);
		ros::param::get("/topic/pointcloud/zed_points", this->pointcloud_topic);
		ros::param::get("/topic/image/lane_mask", this->lane_mask_topic);

		this->odom_sub = node.subscribe(this->odom_topic, 1, 
				&BayesianMapping::get_odom, this);
		this->ground_plane_sub = node.subscribe(this->ground_plane_topic, 1, 
				&BayesianMapping::get_zed_points, this);
		this->lane_mask_sub = node.subscribe(this->lane_mask_topic, 1, 
				&BayesianMapping::get_lane_mask, this);

		this->got_odom = false;
		this->got_zed_points = false;
		this->got_lane_mask = false;

		this->map.header.frame_id = "odom";
		ros::param::get("/map_params/width", this->map_width);
		ros::param::get("/map_params/height", this->map_height);
		ros::param::get("/map_params/resolution", this->map.info.resolution);
		ros::param::get("/map_params/origin/x", this->map.info.origin.position.x);
		ros::param::get("/map_params/origin/y", this->map.info.origin.position.y);
		ros::param::get("/map_params/origin/z", this->map.info.origin.position.z);
		ros::param::get("/map_params/del_confidence/positive");
		ros::param::get("/map_params/del_confidence/positive");
		this->map.info.width = this->map_width;
		this->map.info.height = this->map_height;

	}

private:

	std::string odom_topic;
	std::string ground_plane_topic;
	std::string pointcloud_topic;
	std::string map_topic;
	std::string lane_mask_topic;

	nav_msgs::OccupancyGrid map;
	nav_msgs::Odometry odom;
	sensor_msgs::Image lane_mask;
	sensor_msgs::PointCloud2 zed_points;

	ros::Subscriber odom_sub;
	ros::Subscriber ground_plane_sub;
	ros::Subscriber points_sub;
	ros::Subscriber lane_mask_sub;

	bool got_odom;
	bool got_zed_points;
	bool got_lane_mask;

	int32_t map_width;
	int32_t map_height;

public:

	void run() {
	
		ros::AsyncSpinner spinner(4);
		spinner.start();

		ros::Rate rate(30);

		while (ros::ok()) {

			if (!(this->got_odom && this->got_zed_points && this->got_lane_mask)) {
				std::cerr << this->got_odom << this->got_zed_points << this->got_lane_mask << std::endl;
				continue;
			}

			rate.sleep();
		}
	}

private:

	void get_odom(const nav_msgs::Odometry::ConstPtr &odom_ptr){
		this->odom = *odom_ptr;
		this->got_odom = true;
	}

	void get_zed_points(const sensor_msgs::PointCloud2::ConstPtr &points_ptr){
		this->zed_points = *points_ptr;
		this->got_zed_points = true;
	}

	void get_lane_mask(const sensor_msgs::Image::ConstPtr &mask_ptr){
		this->lane_mask = *mask_ptr;
		this->got_lane_mask = true;
	}

	void update_map(nav_msgs::Odometry odom,
			sensor_msgs::PointCloud2 zed_points,
			sensor_msgs::Image lane_mask) {

		

	}

};

int32_t main(int32_t argc, char **argv) {

	BayesianMapping node(argc, argv);
	node.run();

	return 0;
}




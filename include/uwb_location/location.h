#ifndef RANGE_LOCATION_H
#define RANGE_LOCATION_H

#include "uwb_location/RangeArray.h"
#include "uwb_location/trilateration.h"
#include <Eigen/Eigen>
#include <cmath>
#include <cstdint>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class RangeLocation {
public:
	RangeLocation() {}
	~RangeLocation() {}
	void init(ros::NodeHandle& nh);

private:
	void anchor1Callback_odom(const nav_msgs::Odometry::ConstPtr& odom);
	void anchor2Callback_odom(const nav_msgs::Odometry::ConstPtr& odom);
	void anchor3Callback_odom(const nav_msgs::Odometry::ConstPtr& odom);
	void anchor4Callback_odom(const nav_msgs::Odometry::ConstPtr& odom);

	void anchor1Callback_pose(const geometry_msgs::PoseStamped::ConstPtr& pose);
	void anchor2Callback_pose(const geometry_msgs::PoseStamped::ConstPtr& pose);
	void anchor3Callback_pose(const geometry_msgs::PoseStamped::ConstPtr& pose);
	void anchor4Callback_pose(const geometry_msgs::PoseStamped::ConstPtr& pose);

	void rangeCallback(const uwb_location::RangeArray::ConstPtr& ranges);

	int leastSquaresMethod();

	int trilaterationPosition();

    // 0: trilateration, 1: leastSquares
	int location_method_;

	// 0: fixed anchor, 1: moving anchor
	int anchorMode_;

	// 0: Odometry, 1: PoseStamped
	int pose_type_;
	std::string anchor1_pos_topic_;
	std::string anchor2_pos_topic_;
	std::string anchor3_pos_topic_;
	std::string anchor4_pos_topic_;

	ros::NodeHandle nh_;
	ros::Subscriber anchor1_sub_odom_;
	ros::Subscriber anchor2_sub_odom_;
	ros::Subscriber anchor3_sub_odom_;
	ros::Subscriber anchor4_sub_odom_;
	ros::Subscriber anchor1_sub_pose_;
	ros::Subscriber anchor2_sub_pose_;
	ros::Subscriber anchor3_sub_pose_;
	ros::Subscriber anchor4_sub_pose_;
	ros::Subscriber range_sub_;
	ros::Publisher location_pub_;

	Eigen::Vector3d anchor1_pos_;
	Eigen::Vector3d anchor2_pos_;
	Eigen::Vector3d anchor3_pos_;
	Eigen::Vector3d anchor4_pos_;
	Eigen::Vector4d ranges_;

	Eigen::Vector3d location_;
	geometry_msgs::PoseStamped location_cur_;

	struct num {
		int anc_ID;
		int distance;
	} valid_anc_num[(MAX_ANCHOR_NUMBER + 1)];
};

#endif // RANGE_LOCATION_H
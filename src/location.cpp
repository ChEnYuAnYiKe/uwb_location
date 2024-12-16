#include "uwb_location/location.h"

void RangeLocation::init(ros::NodeHandle& nh) {
	nh.param("location_method", location_method_, 0);
	nh.param("anchorMode", anchorMode_, 0);
	nh.param("pose_type", pose_type_, 0);
	printf("anchorMode: %d, pose_type: %d\n", anchorMode_, pose_type_);
	nh.param("anchor1_pos_topic", anchor1_pos_topic_,
	         std::string("/ugv_1/odom"));
	nh.param("anchor2_pos_topic", anchor2_pos_topic_,
	         std::string("/ugv_2/odom"));
	nh.param("anchor3_pos_topic", anchor3_pos_topic_,
	         std::string("/ugv_3/odom"));
	nh.param("anchor4_pos_topic", anchor4_pos_topic_,
	         std::string("/ugv_4/odom"));

	if (anchorMode_ == 0) {
		nh.param("anchor_1_x", anchor1_pos_(0), -1.0);
		nh.param("anchor_1_y", anchor1_pos_(1), -1.0);
		nh.param("anchor_1_z", anchor1_pos_(2), -1.0);

		nh.param("anchor_2_x", anchor2_pos_(0), -1.0);
		nh.param("anchor_2_y", anchor2_pos_(1), -1.0);
		nh.param("anchor_2_z", anchor2_pos_(2), -1.0);

		nh.param("anchor_3_x", anchor3_pos_(0), -1.0);
		nh.param("anchor_3_y", anchor3_pos_(1), -1.0);
		nh.param("anchor_3_z", anchor3_pos_(2), -1.0);

		nh.param("anchor_4_x", anchor4_pos_(0), -1.0);
		nh.param("anchor_4_y", anchor4_pos_(1), -1.0);
		nh.param("anchor_4_z", anchor4_pos_(2), -1.0);

		printf("anchor1_pos: %f, %f, %f\n", anchor1_pos_(0), anchor1_pos_(1),
		       anchor1_pos_(2));
		printf("anchor2_pos: %f, %f, %f\n", anchor2_pos_(0), anchor2_pos_(1),
		       anchor2_pos_(2));
		printf("anchor3_pos: %f, %f, %f\n", anchor3_pos_(0), anchor3_pos_(1),
		       anchor3_pos_(2));
		printf("anchor4_pos: %f, %f, %f\n", anchor4_pos_(0), anchor4_pos_(1),
		       anchor4_pos_(2));

	} else if (anchorMode_ == 1) {
		if (pose_type_ == 0) {
			anchor1_sub_odom_ =
			    nh.subscribe(anchor1_pos_topic_, 50,
			                 &RangeLocation::anchor1Callback_odom, this);
			anchor2_sub_odom_ =
			    nh.subscribe(anchor2_pos_topic_, 50,
			                 &RangeLocation::anchor2Callback_odom, this);
			anchor3_sub_odom_ =
			    nh.subscribe(anchor3_pos_topic_, 50,
			                 &RangeLocation::anchor3Callback_odom, this);
			anchor4_sub_odom_ =
			    nh.subscribe(anchor4_pos_topic_, 50,
			                 &RangeLocation::anchor4Callback_odom, this);
		} else if (pose_type_ == 1) {
			anchor1_sub_pose_ =
			    nh.subscribe(anchor1_pos_topic_, 50,
			                 &RangeLocation::anchor1Callback_pose, this);
			anchor2_sub_pose_ =
			    nh.subscribe(anchor2_pos_topic_, 50,
			                 &RangeLocation::anchor2Callback_pose, this);
			anchor3_sub_pose_ =
			    nh.subscribe(anchor3_pos_topic_, 50,
			                 &RangeLocation::anchor3Callback_pose, this);
			anchor4_sub_pose_ =
			    nh.subscribe(anchor4_pos_topic_, 50,
			                 &RangeLocation::anchor4Callback_pose, this);
		}
	}

	range_sub_ =
	    nh.subscribe("/uwb/range", 10, &RangeLocation::rangeCallback, this);
	location_pub_ =
	    nh.advertise<geometry_msgs::PoseStamped>("/uwb/location", 10);

	return;
}

void RangeLocation::anchor1Callback_odom(
    const nav_msgs::Odometry::ConstPtr& odom) {
	anchor1_pos_ << odom->pose.pose.position.x, odom->pose.pose.position.y,
	    odom->pose.pose.position.z;
}

void RangeLocation::anchor2Callback_odom(
    const nav_msgs::Odometry::ConstPtr& odom) {
	anchor2_pos_ << odom->pose.pose.position.x, odom->pose.pose.position.y,
	    odom->pose.pose.position.z;
}

void RangeLocation::anchor3Callback_odom(
    const nav_msgs::Odometry::ConstPtr& odom) {
	anchor3_pos_ << odom->pose.pose.position.x, odom->pose.pose.position.y,
	    odom->pose.pose.position.z;
}

void RangeLocation::anchor4Callback_odom(
    const nav_msgs::Odometry::ConstPtr& odom) {
	anchor4_pos_ << odom->pose.pose.position.x, odom->pose.pose.position.y,
	    odom->pose.pose.position.z;
}

void RangeLocation::anchor1Callback_pose(
    const geometry_msgs::PoseStamped::ConstPtr& pose) {
	anchor1_pos_ << pose->pose.position.x, pose->pose.position.y,
	    pose->pose.position.z;
}

void RangeLocation::anchor2Callback_pose(
    const geometry_msgs::PoseStamped::ConstPtr& pose) {
	anchor2_pos_ << pose->pose.position.x, pose->pose.position.y,
	    pose->pose.position.z;
}

void RangeLocation::anchor3Callback_pose(
    const geometry_msgs::PoseStamped::ConstPtr& pose) {
	anchor3_pos_ << pose->pose.position.x, pose->pose.position.y,
	    pose->pose.position.z;
}

void RangeLocation::anchor4Callback_pose(
    const geometry_msgs::PoseStamped::ConstPtr& pose) {
	anchor4_pos_ << pose->pose.position.x, pose->pose.position.y,
	    pose->pose.position.z;
}

void RangeLocation::rangeCallback(
    const uwb_location::RangeArray::ConstPtr& ranges) {
	ranges_ << ranges->data[0] / 1000.0, ranges->data[1] / 1000.0,
	    ranges->data[2] / 1000.0, ranges->data[3] / 1000.0;
	// printf("range1: %f, range2: %f, range3: %f, range4: %f\n",
	// ranges->range1, ranges->range2, ranges->range3, ranges->range4);
	int result;

	ros::Time t1, t2;
	t1 = ros::Time::now();
	if (location_method_ == 0) {
		result = leastSquaresPosition();
	} else if (location_method_ == 1) {
		result = trilaterationPosition();
	}
	t2 = ros::Time::now();
	// printf("leastSquaresPosition compute time: %f\n", (t2 - t1).toSec());

	if (result == UWB_OK) {
		location_cur_.header.stamp = ros::Time::now();
		location_cur_.header.frame_id = "world";
		location_cur_.pose.position.x = location_[0];
		location_cur_.pose.position.y = location_[1];
		location_cur_.pose.position.z = location_[2];
		location_pub_.publish(location_cur_);
	} else {
		ROS_WARN("3D position failed, Error code: %d", result);
	}
}

int RangeLocation::leastSquaresPosition() {
	// 将输入数据存入数组，便于处理
	std::array<Eigen::Vector3d, 4> anchors = {anchor1_pos_, anchor2_pos_,
	                                          anchor3_pos_, anchor4_pos_};
	std::array<double, 4> distances = {ranges_(0), ranges_(1), ranges_(2),
	                                   ranges_(3)};

	// 筛选有效距离和对应的基站
	std::vector<Eigen::Vector3d> valid_anchors;
	std::vector<int64_t> distances_cm;
	for (int i = 0; i < 4; i++) {
		if (distances[i] > 0.0) { // 简单判断有效距离
			distances_cm.push_back(static_cast<int64_t>(distances[i] * 1000.0));
			valid_anchors.push_back(anchors[i] * 1000.0); // m转mm
		}
	}

	int no_valid_distances = valid_anchors.size();
	// 有效基站小于3个无法求解
	if (no_valid_distances < 3) {
		return UWB_ANC_BELOW_THREE;
	}

	// 检查是否所有锚点在同一平面(z相同)
	bool anchors_on_x_y_plane =
	    std::all_of(valid_anchors.begin() + 1, valid_anchors.end(),
	                [&](const Eigen::Vector3d& anchor) {
		                return anchor.z() == valid_anchors[0].z();
	                });

	// 检查线性独立性（2D情况）
	bool lin_dep = true;
	if (no_valid_distances >= 3) {
		for (int i = 2; i < no_valid_distances; i++) {
			Eigen::Vector3d diff_1 = valid_anchors[1] - valid_anchors[0];
			Eigen::Vector3d diff_2 = valid_anchors[i] - valid_anchors[0];
			if (diff_1.cross(diff_2).norm() != 0) {
				lin_dep = false;
				break;
			}
		}
	}

	if (lin_dep && no_valid_distances == 3) {
		return UWB_LIN_DEP_FOR_THREE;
	}

	// 不在同一平面，需4个基站进行3D定位
	if (!anchors_on_x_y_plane && no_valid_distances < 4) {
		return UWB_ANC_ON_ONE_LEVEL;
	}

	// 若不在同一平面，检查三维线性独立性
	if (!anchors_on_x_y_plane) {
		lin_dep = true;
		for (int i = 2; i < no_valid_distances && lin_dep; i++) {
			// 三维线性独立性检查
			if (i != 2) {
				Eigen::Matrix3d mat;
				mat << valid_anchors[0].x(), valid_anchors[0].y(),
				    valid_anchors[0].z(), valid_anchors[1].x(),
				    valid_anchors[1].y(), valid_anchors[1].z(),
				    valid_anchors[i].x(), valid_anchors[i].y(),
				    valid_anchors[i].z();
				if (mat.determinant() == 0) {
					lin_dep = false;
					break;
				}
			}
		}
		if (lin_dep) {
			return UWB_LIN_DEP_FOR_FOUR;
		}
	}

	// 开始最小二乘计算
	double M_11 = 0, M_12 = 0, M_13 = 0, M_22 = 0, M_23 = 0, M_33 = 0;
	double b[3] = {0};
	double x_pos = 0, y_pos = 0, z_pos = 0;

	for (int i = 1; i < no_valid_distances; i++) {
		double dx =
		    static_cast<double>(valid_anchors[i].x() - valid_anchors[0].x());
		double dy =
		    static_cast<double>(valid_anchors[i].y() - valid_anchors[0].y());
		double dz =
		    static_cast<double>(valid_anchors[i].z() - valid_anchors[0].z());

		M_11 += dx * dx;
		M_12 += dx * dy;
		M_13 += dx * dz;
		M_22 += dy * dy;
		M_23 += dy * dz;
		M_33 += dz * dz;

		double temp = static_cast<double>(distances_cm[0] * distances_cm[0]) -
		              static_cast<double>(distances_cm[i] * distances_cm[i]) +
		              (valid_anchors[i].x() * valid_anchors[i].x() +
		               valid_anchors[i].y() * valid_anchors[i].y() +
		               valid_anchors[i].z() * valid_anchors[i].z() -
		               valid_anchors[0].x() * valid_anchors[0].x() -
		               valid_anchors[0].y() * valid_anchors[0].y() -
		               valid_anchors[0].z() * valid_anchors[0].z());

		b[0] += dx * temp;
		b[1] += dy * temp;
		b[2] += dz * temp;
	}

	M_11 *= 2;
	M_12 *= 2;
	M_13 *= 2;
	M_22 *= 2;
	M_23 *= 2;
	M_33 *= 2;

	double denominator = 0, nominator = 0;

	// 求z坐标（若不在同一平面）
	if (!anchors_on_x_y_plane) {
		nominator = b[0] * (M_12 * M_23 - M_13 * M_22) +
		            b[1] * (M_12 * M_13 - M_11 * M_23) +
		            b[2] * (M_11 * M_22 - M_12 * M_12);
		denominator = M_11 * (M_33 * M_22 - M_23 * M_23) +
		              2 * M_12 * M_13 * M_23 - M_33 * M_12 * M_12 -
		              M_22 * M_13 * M_13;

		if (denominator == 0) {
			return UWB_RANK_ZERO;
		}
		z_pos = (nominator * 10 / denominator + 5) / 10;
	} else {
		z_pos = 0;
	}

	// 求y_pos
	nominator =
	    b[1] * M_11 - b[0] * M_12 - (z_pos * (M_11 * M_23 - M_12 * M_13));
	denominator = M_11 * M_22 - M_12 * M_12;
	if (denominator == 0) {
		return UWB_RANK_ZERO;
	}
	y_pos = (nominator * 10 / denominator + 5) / 10;

	// 求x_pos
	nominator = b[0] - z_pos * M_13 - y_pos * M_12;
	denominator = M_11;
	if (denominator == 0) {
		return UWB_RANK_ZERO;
	}
	x_pos = (nominator * 10 / denominator + 5) / 10;

	// 若在同一平面，根据x,y再估计z
	if (anchors_on_x_y_plane) {
		double z_sum = 0;
		int count_pos = 0;
		for (int i = 0; i < no_valid_distances; i++) {
			double dx = x_pos - static_cast<double>(valid_anchors[i].x());
			double dy = y_pos - static_cast<double>(valid_anchors[i].y());
			double temp =
			    static_cast<double>(distances_cm[i] * distances_cm[i]) -
			    dx * dx - dy * dy;
			if (temp >= 0) {
				double z_calc = sqrt(temp);
				z_sum += z_calc;
				count_pos++;
			}
		}
		if (count_pos > 0) {
			z_pos = z_sum / count_pos + valid_anchors[0].z();
		} else {
			z_pos = valid_anchors[0].z();
		}
	}

	location_ = Eigen::Vector3d(x_pos / 1000.0, y_pos / 1000.0, z_pos / 1000.0);
	return UWB_OK;
}

int RangeLocation::trilaterationPosition() {
	vec3d o1, o2, p1, p2, p3, p4, best_solution;
	double r1 = 0, r2 = 0, r3 = 0, r4 = 0, best_3derror, best_gdoprate;
	int result;
	int error, combination;
	int valid_anc_count = 0;
	int use3anc = 0;

	std::array<Eigen::Vector3d, 4> anchor_array = {anchor1_pos_, anchor2_pos_,
	                                               anchor3_pos_, anchor4_pos_};

	for (int i = 0; i < (MAX_ANCHOR_NUMBER + 1); i++) // 清空结构体数组
	{
		valid_anc_num[i].anc_ID = 0;
		valid_anc_num[i].distance = -1;
	}

	for (int i = 0; i < MAX_ANCHOR_NUMBER; i++) // 验证几个有效距离值
	{
		if (ranges_(i) > 0) // 如果测量到的距离存在则有效
		{
			valid_anc_num[valid_anc_count].anc_ID = i; // 记录有效基站编号
			valid_anc_num[valid_anc_count].distance =
			    ranges_(i); // 记录有效基站距离
			valid_anc_count++;
		}
	}
	// printf("valid_anc number = %d\n", valid_anc_count);

	if (valid_anc_count < 3) {
		return -1;
		ROS_WARN("valid_anc_count < 3 !");
	} else if (valid_anc_count == 3) // 执行三基站定位
	{
		use3anc = 1;
		/* Anchors coordinate */
		p1.x = anchor_array[valid_anc_num[0].anc_ID].x();
		p1.y = anchor_array[valid_anc_num[0].anc_ID].y();
		p1.z = anchor_array[valid_anc_num[0].anc_ID].z();
		p2.x = anchor_array[valid_anc_num[1].anc_ID].x();
		p2.y = anchor_array[valid_anc_num[1].anc_ID].y();
		p2.z = anchor_array[valid_anc_num[1].anc_ID].z();
		p3.x = anchor_array[valid_anc_num[2].anc_ID].x();
		p3.y = anchor_array[valid_anc_num[2].anc_ID].y();
		p3.z = anchor_array[valid_anc_num[2].anc_ID].z();
		p4.x = p1.x;
		p4.y = p1.y;
		p4.z = p1.z;

		r1 = ranges_(valid_anc_num[0].anc_ID);
		r2 = ranges_(valid_anc_num[1].anc_ID);
		r3 = ranges_(valid_anc_num[2].anc_ID);
		r4 = r1;
	} else if (valid_anc_count == 4) // 执行4基站定位
	{
		/* Anchors coordinate */
		p1.x = anchor_array[valid_anc_num[0].anc_ID].x();
		p1.y = anchor_array[valid_anc_num[0].anc_ID].y();
		p1.z = anchor_array[valid_anc_num[0].anc_ID].z();
		p2.x = anchor_array[valid_anc_num[1].anc_ID].x();
		p2.y = anchor_array[valid_anc_num[1].anc_ID].y();
		p2.z = anchor_array[valid_anc_num[1].anc_ID].z();
		p3.x = anchor_array[valid_anc_num[2].anc_ID].x();
		p3.y = anchor_array[valid_anc_num[2].anc_ID].y();
		p3.z = anchor_array[valid_anc_num[2].anc_ID].z();
		p4.x = anchor_array[valid_anc_num[3].anc_ID].x();
		p4.y = anchor_array[valid_anc_num[3].anc_ID].y();
		p4.z = anchor_array[valid_anc_num[3].anc_ID].z();

		r1 = ranges_(valid_anc_num[0].anc_ID);
		r2 = ranges_(valid_anc_num[1].anc_ID);
		r3 = ranges_(valid_anc_num[2].anc_ID);
		r4 = ranges_(valid_anc_num[3].anc_ID);
	}

	result = deca_3dlocate(&o1, &o2, &best_solution, &error, &best_3derror,
	                       &best_gdoprate, p1, r1, p2, r2, p3, r3, p4, r4,
	                       &combination);
	if (result == 0) {
		ROS_WARN("deca_3dlocate failed !");
	} else if (result >= 0) {
		if (o1.z >= o2.z)
			best_solution.z = o1.z;
		else
			best_solution.z = o2.z;

		if (use3anc == 1 || result == TRIL_3SPHERES) {
			if (o1.z > p1.z)
				best_solution = o1;
			else
				best_solution = o2;
		}
		// assume tag is higher than the anchors
		location_ =
		    Eigen::Vector3d(best_solution.x, best_solution.y, best_solution.z);
		return UWB_OK;
	}
	return result;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "uwb_location_node");
	ros::NodeHandle nh("~");

	RangeLocation range_location;
	range_location.init(nh);

	ROS_INFO("Starting uwb location!");
	ros::spin();
	return 0;
}
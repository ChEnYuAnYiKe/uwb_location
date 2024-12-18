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
		// result = leastSquaresPosition();
		result = leastSquaresMethod();
	} else if (location_method_ == 1) {
		result = trilaterationPosition();
	}
	t2 = ros::Time::now();
	// printf("leastSquaresPosition compute time: %f\n", (t2 - t1).toSec());

	// printf("[location] x: %f, y: %f, z:%f\n", location_[0], location_[1],
	//        location_[2]);

	if (result == UWB_OK) {
		location_cur_.header.stamp = ros::Time::now();
		location_cur_.header.frame_id = "world";
		location_cur_.pose.position.x = location_(0);
		location_cur_.pose.position.y = location_(1);
		location_cur_.pose.position.z = location_(2);
		location_pub_.publish(location_cur_);
	} else {
		ROS_WARN("3D position failed, Error code: %d", result);
	}
}

// int RangeLocation::leastSquaresPosition() {
// 	// 将输入数据存入数组，便于处理
// 	std::array<Eigen::Vector3d, 4> anchors = {anchor1_pos_, anchor2_pos_,
// 	                                          anchor3_pos_, anchor4_pos_};
// 	std::array<double, 4> distances = {ranges_(0), ranges_(1), ranges_(2),
// 	                                   ranges_(3)};

// 	// 筛选有效距离和对应的基站
// 	std::vector<Eigen::Vector3d> valid_anchors;
// 	std::vector<int64_t> distances_cm;
// 	for (int i = 0; i < 4; i++) {
// 		if (distances[i] > 0.0) { // 简单判断有效距离
// 			distances_cm.push_back(static_cast<int64_t>(distances[i] * 1000.0));
// 			valid_anchors.push_back(anchors[i] * 1000.0); // m转mm
// 		}
// 	}
// }

int RangeLocation::leastSquaresMethod() {
	/*!@brief: This function calculates the 3D position of the initiator from
	 the anchor distances and positions using least squared errors. * The
	 function expects more than 4 anchors. The used equation system looks like
	 follows:\n \verbatim -					- | M_11	M_12	M_13 |	 x b[0]
	           | M_21	M_22	M_23 | * y	= b[1]
	           | M_31	M_32	M_33 |	 z	  b[2]
	            -					-
	 \endverbatim
	 * @param distances_cm_in_pt: 			Pointer to array that contains the
	 distances to the anchors in cm (including invalid results)
	 * @param no_distances: 				Number of valid distances in
	 distance array (it's not the size of the array)
	 * @param anchor_pos: 	                Pointer to array that contains
	 anchor positions in cm (including positions related to invalid results)
	 * @param no_anc_positions: 			Number of valid anchor positions in
	 the position array (it's not the size of the array)
	 * @param position_result_pt: 			Pointer toposition. position_t
	 variable that holds the result of this calculation
	 * @return: The function returns a status code. */

	/* 		Algorithm used:
	 *		Linear Least Sqaures to solve Multilateration
	 * 		with a Special case if there are only 3 Anchors.
	 * 		Output is the Coordinates of the Initiator in relation to Anchor 0
	 *		in NEU (North-East-Up) Framing In cm
	 */

	/* Resulting Position Vector*/
	double x_pos = 0;
	double y_pos = 0;
	double z_pos = 0;
	/* Matrix components (3*3 Matrix resulting from least square error method)
	 * [cm^2] */
	double M_11 = 0;
	double M_12 = 0; // = M_21
	double M_13 = 0; // = M_31
	double M_22 = 0;
	double M_23 = 0; // = M_23
	double M_33 = 0;

	/* Vector components (3*1 Vector resulting from least square error method)
	 * [cm^3] */
	double b[3] = {0};

	/* Miscellaneous variables */
	double temp = 0;
	double temp2 = 0;
	double nominator = 0;
	double denominator = 0;
	bool anchors_on_x_y_plane =
	    true; // Is true, if all anchors are on the same height => x-y-plane
	bool lin_dep =
	    true; // All vectors are linear dependent, if this variable is true
	uint8_t ind_y_indi =
	    0; // numberr of independet vectors
	       // // First anchor index, for which the second row entry of the
	       // matrix [(x_1 - x_0) (x_2 - x_0) ... ; (y_1 - x_0) (y_2 - x_0) ...]
	       // is non-zero => linear independent

	/* Arrays for used distances and anchor positions (without rejected ones) */
	uint8_t no_distances = 4;
	int distances_cm[no_distances];
	position_t anchor_pos[no_distances]; // position in CM
	uint8_t no_valid_distances = 0;

	/* Reject invalid distances (including related anchor position) */
	std::array<Eigen::Vector3d, 4> anchor_array = {anchor1_pos_, anchor2_pos_,
	                                          anchor3_pos_, anchor4_pos_};

	for (int i = 0; i < no_distances; i++) {
		if (ranges_(i) > 0) {
			// excludes any distance that is 0xFFFFU (int16 Maximum Value)
			distances_cm[no_valid_distances] = ranges_(i) * 100;
			anchor_pos[no_valid_distances].x = anchor_array[no_valid_distances].x() * 100;
			anchor_pos[no_valid_distances].y = anchor_array[no_valid_distances].y() * 100;
			anchor_pos[no_valid_distances].z = anchor_array[no_valid_distances].z() * 100;
			no_valid_distances++;
		}
		// if (_dis[i] != -1)
		// {
		// 	//excludes any distance that is 0xFFFFU (int16 Maximum Value)
		// 	distances_cm[no_valid_distances] = _dis[i];
		// 	anchor_pos[no_valid_distances] = _ancarray[i];
		//     no_valid_distances++;
		// }
		else {
			// printf("%d = -1\n", i);
		}
	}

	// printf("no_valid_distances = %d\n", no_valid_distances);

	/* Check, if there are enough valid results for doing the localization at
	 * all */
	if (no_valid_distances < 3) {
		return UWB_ANC_BELOW_THREE;
	}

	/* Check, if anchors are on the same x-y plane */
	for (int i = 1; i < no_valid_distances; i++) {
		if (anchor_pos[i].z != anchor_pos[0].z) {
			anchors_on_x_y_plane = false;
			break;
		}
	}

	/**** Check, if there are enough linear independent anchor positions ****/

	/* Check, if the matrix |(x_1 - x_0) (x_2 - x_0) ... | has rank 2
	 * 			            |(y_1 - y_0) (y_2 - y_0) ... | 				*/

	for (ind_y_indi = 2;
	     ((ind_y_indi < no_valid_distances) && (lin_dep == true));
	     ind_y_indi++) {
		temp = ((int64_t)anchor_pos[ind_y_indi].y - (int64_t)anchor_pos[0].y) *
		       ((int64_t)anchor_pos[1].x - (int64_t)anchor_pos[0].x);
		temp2 = ((int64_t)anchor_pos[1].y - (int64_t)anchor_pos[0].y) *
		        ((int64_t)anchor_pos[ind_y_indi].x - (int64_t)anchor_pos[0].x);

		if ((temp - temp2) != 0) {
			lin_dep = false;
			break;
		}
	}

	/* Leave function, if rank is below 2 */
	if (lin_dep == true) {
		return UWB_LIN_DEP_FOR_THREE;
	}

	/* If the anchors are not on the same plane, three vectors must be
	 * independent => check */
	if (!anchors_on_x_y_plane) {
		/* Check, if there are enough valid results for doing the localization
		 */
		if (no_valid_distances < 4) {
			return UWB_ANC_ON_ONE_LEVEL;
		}

		/* Check, if the matrix |(x_1 - x_0) (x_2 - x_0) (x_3 - x_0) ... | has
		 * rank 3 (Rank y, y already checked)
		 * 			            |(y_1 - y_0) (y_2 - y_0) (y_3 - y_0) ... |
		 * 			            |(z_1 - z_0) (z_2 - z_0) (z_3 - z_0) ... |
		 */
		lin_dep = true;

		for (int i = 2; ((i < no_valid_distances) && (lin_dep == true)); i++) {
			if (i != ind_y_indi) {
				/* (x_1 - x_0)*[(y_2 - y_0)(z_n - z_0) - (y_n - y_0)(z_2 - z_0)]
				 */
				temp = ((int64_t)anchor_pos[ind_y_indi].y -
				        (int64_t)anchor_pos[0].y) *
				       ((int64_t)anchor_pos[i].z - (int64_t)anchor_pos[0].z);
				temp -= ((int64_t)anchor_pos[i].y - (int64_t)anchor_pos[0].y) *
				        ((int64_t)anchor_pos[ind_y_indi].z -
				         (int64_t)anchor_pos[0].z);
				temp2 = ((int64_t)anchor_pos[1].x - (int64_t)anchor_pos[0].x) *
				        temp;

				/* Add (x_2 - x_0)*[(y_n - y_0)(z_1 - z_0) - (y_1 - y_0)(z_n -
				 * z_0)] */
				temp = ((int64_t)anchor_pos[i].y - (int64_t)anchor_pos[0].y) *
				       ((int64_t)anchor_pos[1].z - (int64_t)anchor_pos[0].z);
				temp -= ((int64_t)anchor_pos[1].y - (int64_t)anchor_pos[0].y) *
				        ((int64_t)anchor_pos[i].z - (int64_t)anchor_pos[0].z);
				temp2 += ((int64_t)anchor_pos[ind_y_indi].x -
				          (int64_t)anchor_pos[0].x) *
				         temp;

				/* Add (x_n - x_0)*[(y_1 - y_0)(z_2 - z_0) - (y_2 - y_0)(z_1 -
				 * z_0)] */
				temp = ((int64_t)anchor_pos[1].y - (int64_t)anchor_pos[0].y) *
				       ((int64_t)anchor_pos[ind_y_indi].z -
				        (int64_t)anchor_pos[0].z);
				temp -= ((int64_t)anchor_pos[ind_y_indi].y -
				         (int64_t)anchor_pos[0].y) *
				        ((int64_t)anchor_pos[1].z - (int64_t)anchor_pos[0].z);
				temp2 += ((int64_t)anchor_pos[i].x - (int64_t)anchor_pos[0].x) *
				         temp;

				if (temp2 != 0) {
					lin_dep = false;
				}
			}
		}

		/* Leave function, if rank is below 3 */
		if (lin_dep == true) {
			return UWB_LIN_DEP_FOR_FOUR;
		}
	}

	/************************************************** Algorithm
	 * ***********************************************************************/

	/* Writing values resulting from least square error method (A_trans*A*x =
	 * A_trans*r; row 0 was used to remove x^2,y^2,z^2 entries => index starts
	 * at 1) */
	for (int i = 1; i < no_valid_distances; i++) {
		/* Matrix (needed to be multiplied with 2, afterwards) */
		M_11 += (int64_t)pow((int64_t)(anchor_pos[i].x - anchor_pos[0].x), 2);
		M_12 += (int64_t)((int64_t)(anchor_pos[i].x - anchor_pos[0].x) *
		                  (int64_t)(anchor_pos[i].y - anchor_pos[0].y));
		M_13 += (int64_t)((int64_t)(anchor_pos[i].x - anchor_pos[0].x) *
		                  (int64_t)(anchor_pos[i].z - anchor_pos[0].z));
		M_22 += (int64_t)pow((int64_t)(anchor_pos[i].y - anchor_pos[0].y), 2);
		M_23 += (int64_t)((int64_t)(anchor_pos[i].y - anchor_pos[0].y) *
		                  (int64_t)(anchor_pos[i].z - anchor_pos[0].z));
		M_33 += (int64_t)pow((int64_t)(anchor_pos[i].z - anchor_pos[0].z), 2);

		/* Vector */
		temp = (int64_t)((int64_t)pow(distances_cm[0], 2) -
		                 (int64_t)pow(distances_cm[i], 2) +
		                 (int64_t)pow(anchor_pos[i].x, 2) +
		                 (int64_t)pow(anchor_pos[i].y, 2) +
		                 (int64_t)pow(anchor_pos[i].z, 2) -
		                 (int64_t)pow(anchor_pos[0].x, 2) -
		                 (int64_t)pow(anchor_pos[0].y, 2) -
		                 (int64_t)pow(anchor_pos[0].z, 2));

		b[0] += (int64_t)((int64_t)(anchor_pos[i].x - anchor_pos[0].x) * temp);
		b[1] += (int64_t)((int64_t)(anchor_pos[i].y - anchor_pos[0].y) * temp);
		b[2] += (int64_t)((int64_t)(anchor_pos[i].z - anchor_pos[0].z) * temp);
	}

	M_11 = 2 * M_11;
	M_12 = 2 * M_12;
	M_13 = 2 * M_13;
	M_22 = 2 * M_22;
	M_23 = 2 * M_23;
	M_33 = 2 * M_33;

	/* Calculating the z-position, if calculation is possible (at least one
	 * anchor at z != 0) */
	if (anchors_on_x_y_plane == false) {
		nominator = b[0] * (M_12 * M_23 - M_13 * M_22) +
		            b[1] * (M_12 * M_13 - M_11 * M_23) +
		            b[2] * (M_11 * M_22 - M_12 * M_12); // [cm^7]
		denominator = M_11 * (M_33 * M_22 - M_23 * M_23) +
		              2 * M_12 * M_13 * M_23 - M_33 * M_12 * M_12 -
		              M_22 * M_13 * M_13; // [cm^6]

		/* Check, if denominator is zero (Rank of matrix not high enough) */
		if (denominator == 0) {
			return UWB_RANK_ZERO;
		}

		z_pos = ((nominator * 10) / denominator + 5) / 10; // [cm]
	}

	/* Else prepare for different calculation approach (after x and y were
	   calculated) */
	else {
		z_pos = 0;
	}

	/* Calculating the y-position */
	nominator = b[1] * M_11 - b[0] * M_12 -
	            (z_pos * (M_11 * M_23 - M_12 * M_13)); // [cm^5]
	denominator = M_11 * M_22 - M_12 * M_12;           // [cm^4]

	/* Check, if denominator is zero (Rank of matrix not high enough) */
	if (denominator == 0) {
		return UWB_RANK_ZERO;
	}

	y_pos = ((nominator * 10) / denominator + 5) / 10; // [cm]

	/* Calculating the x-position */
	nominator = b[0] - z_pos * M_13 - y_pos * M_12; // [cm^3]
	denominator = M_11;                             // [cm^2]

	x_pos = ((nominator * 10) / denominator + 5) / 10; // [cm]

	/* Calculate z-position form x and y coordinates, if z can't be determined
	 * by previous steps (All anchors at z_n = 0) */
	if (anchors_on_x_y_plane == true) {
		/* Calculate z-positon relative to the anchor grid's height */
		// for (int i = 0; i < no_distances; i++) {
		for (int i = 0; i < no_valid_distances; i++) {
			/* z² = dis_meas_n² - (x - x_anc_n)² - (y - y_anc_n)² */
			temp =
			    (int64_t)((int64_t)pow(distances_cm[i], 2) -
			              (int64_t)pow((x_pos - (int64_t)anchor_pos[i].x), 2) -
			              (int64_t)pow((y_pos - (int64_t)anchor_pos[i].y), 2));

			/* z² must be positive, else x and y must be wrong => calculate
			 * positive sqrt and sum up all calculated heights, if positive */
			if (temp >= 0) {
				z_pos += (int64_t)sqrt(temp);

			} else {
				z_pos = 0;
			}
		}

		// printf("z_pos1 = %ld\n", z_pos);

		// z_pos = z_pos / no_distances;	// Divide sum by number of distances
		// to get the average
		z_pos = z_pos / no_valid_distances; // Divide sum by number of distances
		                                    // to get the average

		/// printf("z_pos2 = %ld\n", z_pos);

		/* Add height of the anchor grid's height */
		z_pos += anchor_pos[0].z;
	}

	location_(0) = (double)x_pos / 100.0;
	location_(1) = (double)y_pos / 100.0;
	location_(2) = (double)z_pos / 100.0;

	// printf("x=%f, y=%f, z=%f\n",x_pos, y_pos, z_pos);

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
		location_ << best_solution.x, best_solution.y, best_solution.z;
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
#include "autoposition.h"
#include "std_msgs/String.h"
#include "trilateration.h"
#include "uwb_location/uwb.h"
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <string.h>
#include <string>
#include <boost/thread/shared_mutex.hpp>

#define MAX_DATA_NUM 1024 // 传消息内容最大长度
#define DataHead 'm'
#define DataHead2 'M'
#define DataTail '\n'

unsigned char BufDataFromCtrl[MAX_DATA_NUM];
int BufCtrlPosit_w = 0;
int BufCtrlPosit_r = 0;
int DataRecord = 0, rcvsign = 0;

Eigen::MatrixXd anchorArray = Eigen::MatrixXd::Zero(8, 3);
Eigen::MatrixXd anchorArray_last = Eigen::MatrixXd::Zero(8, 3);

boost::shared_mutex anchorArrayMutex_;

ros::Subscriber anchor1_pos_sub;
ros::Subscriber anchor2_pos_sub;
ros::Subscriber anchor3_pos_sub;
ros::Subscriber anchor4_pos_sub;

// std::string anchor1_pos_topic = "/vrpn_client_node/tb0/pose";
// std::string anchor2_pos_topic = "/vrpn_client_node/tb1/pose";
// std::string anchor3_pos_topic = "/vrpn_client_node/tb2/pose";
// std::string anchor4_pos_topic = "/vrpn_client_node/tb3/pose";
std::string anchor1_pos_topic = "/robot_0/Odometry";
std::string anchor2_pos_topic = "/robot_1/Odometry";
std::string anchor3_pos_topic = "/robot_2/Odometry";
std::string anchor4_pos_topic = "/robot_3/Odometry";

// 是否开启自标定模式
int AutopositionMode;
// 标签定位算法：1 使用三边定位法；2 使用最小二乘法
int TagpositionMode;

std::string order_start = "$ancrangestart\r\n";
std::string order_stop = "$ancrangestop\r\n";

// 输出的位置变量
vec3d report;

void receive_deal_func(serial::Serial& sp);

void CtrlSerDataDeal(serial::Serial& sp);

void anchor1_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

void anchor2_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

void anchor3_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

void anchor4_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

void anchor1_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

void anchor2_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

void anchor3_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

void anchor4_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char** argv);

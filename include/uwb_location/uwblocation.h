#include "autoposition.h"
#include "trilateration.h"

#include <Eigen/Core>
// #include <string>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <boost/thread/shared_mutex.hpp>

#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#define MAX_DATA_NUM 1024 // 传消息内容最大长度
#define DataHead 'm'
#define DataHead2 'M'
#define DataTail '\n'

class Uwblocator
{
private:
    unsigned char BufDataFromCtrl[MAX_DATA_NUM];
    int BufCtrlPosit_w;
    int BufCtrlPosit_r;
    int DataRecord, rcvsign;

    unsigned char receive_buf[3000];
    int result;
    bool isAutoposition;

    // Eigen::MatrixXd anchorArray = Eigen::MatrixXd::Zero(8, 3);
    // Eigen::MatrixXd anchorArray_last = Eigen::MatrixXd::Zero(8, 3);

    boost::shared_mutex anchorArrayMutex_;

    ros::Subscriber anchor1_pos_sub;
    ros::Subscriber anchor2_pos_sub;
    ros::Subscriber anchor3_pos_sub;
    ros::Subscriber anchor4_pos_sub;

    ros::Publisher uwb_pub;

    ros::Timer compute_pos_timer;

    std::string anchor1_pos_topic;
    std::string anchor2_pos_topic;
    std::string anchor3_pos_topic;
    std::string anchor4_pos_topic;
	std::string port_name;
	// std::string anchor1_pos_topic = "/robot_0/Odometry";
	// std::string anchor2_pos_topic = "/robot_1/Odometry";
	// std::string anchor3_pos_topic = "/robot_2/Odometry";
	// std::string anchor4_pos_topic = "/robot_3/Odometry";
	int height_extra;

	// 0_fixed_anchor_pos, 2_moving_anchors (1_auto_position, abandoned)
    int AnchorMode;
    // 1_trilaterationMethod, 2_leastSquaresMethod
    int TagpositionMode;

    int pub_rate;
    double pub_rate_inv;

    std::string order_start;
    std::string order_stop;

    // 串口连接类
    serial::Serial sp;

    // 定位功能类
    Trilateration trilaterator;

    // 输出的位置变量
    vec3d report;

    // 发布uwb话题
	geometry_msgs::PoseStamped uwb_data;

    void CtrlSerDataDeal(size_t len);
    void receive_deal_func();

    void anchor1_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void anchor2_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void anchor3_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void anchor4_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    void fix_anchor_callback(const ros::TimerEvent& event);
    void dynamic_anchor_callback(const ros::TimerEvent& event);

public:
    void init(ros::NodeHandle &nh);
    void run();
    // int main(int argc, char** argv);
};

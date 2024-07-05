#include "autoposition.h"
#include "std_msgs/String.h"
#include "trilateration.h"
#include "uwb_location/uwb.h"
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <string.h>
#include <string>

#define MAX_DATA_NUM 1024 // 传消息内容最大长度
#define DataHead 'm'
#define DataHead2 'M'
#define DataTail '\n'

class uwbPositionSystem {
public:
    uwbPositionSystem(ros::NodeHandle &nh, serial::Serial& sp);
    ~uwbPositionSystem();

    // 输出的位置变量
    vec3d report;

    ros::Publisher uwb_publisher;

    void CtrlSerDataDeal(serial::Serial& sp);

    void processSerialData(serial::Serial& sp);

private:
    // 串口相关变量
    unsigned char receive_buf[3000];  
    unsigned char BufDataFromCtrl[MAX_DATA_NUM];
    int BufCtrlPosit_w;
    int BufCtrlPosit_r;
    int DataRecord, rcvsign;

    // 是否开启自标定模式
    int AutopositionMode;
    // 标签定位算法：1 使用三边定位法；2 使用最小二乘法
    int TagpositionMode;

    bool isAutoposition; 

    int result;

    Eigen::MatrixXd anchorArray;
    Eigen::MatrixXd anchorArray_last;

    std::string order_start;
    std::string order_stop;

    std::string anchor1_pos_topic;
	std::string anchor2_pos_topic;
	std::string anchor3_pos_topic;
	std::string anchor4_pos_topic;

    ros::Subscriber anchor1_pos_sub;
    ros::Subscriber anchor2_pos_sub;
    ros::Subscriber anchor3_pos_sub;
    ros::Subscriber anchor4_pos_sub;

    ros::Publisher array_row1_pub;
    ros::Publisher array_row2_pub;
    ros::Publisher array_row3_pub;
    ros::Publisher array_row4_pub;

    geometry_msgs::PoseStamped array_row1;
    geometry_msgs::PoseStamped array_row2;
    geometry_msgs::PoseStamped array_row3;
    geometry_msgs::PoseStamped array_row4;

    void receive_deal_func(serial::Serial& sp);

	void array_pub_wrapper(Eigen::MatrixXd& anchorArray);

	void anchor1_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void anchor2_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void anchor3_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void anchor4_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};


// int main(int argc, char** argv);

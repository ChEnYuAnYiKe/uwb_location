#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <string.h>
#include <Eigen/Core>

#include "std_msgs/String.h"       
#include <sensor_msgs/Imu.h>    

#include "uwb_location/uwb.h"
#include "trilateration.h"
#include "autoposition.h"

#define MAX_DATA_NUM	1024	//传消息内容最大长度
#define DataHead        'm'       
#define DataHead2        'M'   
#define DataTail        '\n'   

unsigned char BufDataFromCtrl[MAX_DATA_NUM];
int BufCtrlPosit_w = 0;
int BufCtrlPosit_r = 0;
int DataRecord=0, rcvsign = 0;

Eigen::MatrixXd anchorArray = Eigen::MatrixXd::Zero(8, 3);
Eigen::MatrixXd anchorArray_last = Eigen::MatrixXd::Zero(8, 3);

// 是否开启自标定模式
bool AutopositionMode;
// 标签定位算法：1 使用三边定位法；2 使用最小二乘法
int TagpositionMode;

std::string order_start = "$ancrangestart\r\n";
std::string order_stop = "$ancrangestop\r\n";

// 输出的位置变量
vec3d report;

void receive_deal_func(serial::Serial& sp);

void CtrlSerDataDeal(serial::Serial& sp);

int main(int argc, char** argv);

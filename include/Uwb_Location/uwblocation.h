#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <string.h>
#include <Eigen/Core>

#include "std_msgs/String.h"       
#include <sensor_msgs/Imu.h>    

#include "Uwb_Location/uwb.h"
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

void receive_deal_func(serial::Serial& sp);

void CtrlSerDataDeal(serial::Serial& sp);

int main(int argc, char** argv);

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

/*
// init the coords of the anchors
//A0 uint:m
anchorArray[0].x = 0.0; 
anchorArray[0].y = 0.0; 
anchorArray[0].z = 3.0; 
//A1 uint:m
anchorArray[1].x = 3.0; 
anchorArray[1].y = 0.0; 
anchorArray[1].z = 2.18;
//A2 uint:m
anchorArray[2].x = 3.0; 
anchorArray[2].y = 1.6; 
anchorArray[2].z = 2.18; 
//A3 uint:m
anchorArray[3].x = 0.0; 
anchorArray[3].y = 1.6; 
anchorArray[3].z = 2.18; 

//A4 uint:m
anchorArray[4].x = 2.0; 
anchorArray[4].y = 1.0; 
anchorArray[4].z = 2.5; 
//A5 uint:m
anchorArray[5].x = 2.0; 
anchorArray[5].y = 0.0; 
anchorArray[5].z = 2.5;
//A6 uint:m
anchorArray[6].x = 3.0; 
anchorArray[6].y = 1.0; 
anchorArray[6].z = 2.5; 
//A7 uint:m
anchorArray[7].x = 3.0; 
anchorArray[7].y = 0.0; 
anchorArray[7].z = 2.5; 
*/

#define MAX_DATA_NUM	1024	//传消息内容最大长度
#define DataHead        'm'       
#define DataHead2        'M'   
#define DataTail        '\n'   
unsigned char BufDataFromCtrl[MAX_DATA_NUM];
int BufCtrlPosit_w = 0, BufCtrlPosit_r = 0;
int DataRecord=0, rcvsign = 0;

bool autoPosRunning = false;

void receive_deal_func();

void CtrlSerDataDeal();

int main(int argc, char** argv);

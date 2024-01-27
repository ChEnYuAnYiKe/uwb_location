#include "Uwb_Location/UwbLocation.h"

using namespace std;

unsigned char receive_buf[300] = {0};
vec3d report;
Eigen::MatrixXd anchorArray = Eigen::MatrixXd::Zero(8, 3);
Quaternion q;
Quaternion Q;
int result = 0; 
float velocityac[3],angleac[3];
bool isAutoposition = false;

void receive_deal_func()
{
    
    int range[8] = {-1};

    //  'mc' tag to anchor range bias corrected ranges – used for tag location
    if((receive_buf[0] == 'm') && (receive_buf[1] == 'c'))
    {
        int aid, tid, lnum, seq, mask;
        int rangetime;
        char role;
        int data_len = strlen((char*)receive_buf);
        //printf("lenmc = %d\n", data_len);
        if(data_len == 106)
        {
            int n = sscanf((char*)receive_buf,"mc %x %x %x %x %x %x %x %x %x %x %x %x %c%d:%d", 
                            &mask, 
                            &range[0], &range[1], &range[2], &range[3], 
                            &range[4], &range[5], &range[6], &range[7],
                            &lnum, &seq, &rangetime, &role, &tid, &aid);
            printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d(mm)\nrange[3]=%d(mm)\nrange[4]=%d(mm)\nrange[5]=%d(mm)\nrange[6]=%d(mm)\nrange[7]=%d(mm)\r\n",
                                mask, range[0], range[1], range[2], range[3], 
                                      range[4], range[5], range[6], range[7]);
        }
        else if(data_len == 70)
        {
            int n = sscanf((char*)receive_buf,"mc %x %x %x %x %x %x %x %x %c%d:%d", 
                            &mask, 
                            &range[0], &range[1], &range[2], &range[3], 
                            &lnum, &seq, &rangetime, &role, &tid, &aid);
            printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d(mm)\nrange[3]=%d(mm)\r\n",
                                mask, range[0], range[1], range[2], range[3]);
        }
        else
        {
            return;
        }

        result = GetLocation(&report, anchorArray, &range[0]);

        printf("result = %d\n",result);
        printf("x = %f\n",report.x);
        printf("y = %f\n",report.y);
        printf("z = %f\n",report.z);

        return;
    }
    // 'ma' anchor to anchor range bias corrected ranges – used for anchor auto-positioning
    else if ((receive_buf[0] == 'm') && (receive_buf[1] == 'a'))
    {
        int aid, tid, lnum, seq, mask;
        int rangetime;
        char role;
        int rx_power;
        int data_len = strlen((char*)receive_buf);

        if((data_len == 106))
        {
            autoPosRunning = true;
            int n = sscanf((char*)receive_buf,"ma %x %x %x %x %x %x %x %x %x %x %x %x %c%d:%d %x",
                           &mask,
                           &range[0], &range[1],
                           &range[2], &range[3],
                           &range[4], &range[5],
                           &range[6], &range[7],
                           &lnum, &seq, &rangetime, &role, &tid, &aid, &rx_power);
            printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d(mm)\nrange[3]=%d(mm)\nrole=%c:%d\r\n",
                                mask, range[0], range[1], range[2], range[3], role, aid);
            
            if(n == EOF)
            {
                ROS_ERROR_STREAM("data Error !!!!!!!!!!!!!!!!");
                return;
            }
        }
        else
        {
            ROS_ERROR_STREAM("data Error !!!!!!!!!!!!!!!!");
            return;
        }
        // 对收集到的基站间距离进行自定位，获得定位后的坐标
        // ATTENTION：该自定位算法只能定位各个基站的二维坐标(x,y)，z轴的坐标需要给定
        autopositioning(&range[0], aid, anchorArray);
        printf("[Autoposition] A0:(%d, %d, %d),A1:(%d, %d, %d),A2:(%d, %d, %d),A3:(%d, %d, %d)\n",
                anchorArray(0,0), anchorArray(0,1), anchorArray(0,2),
                anchorArray(1,0), anchorArray(1,1), anchorArray(1,2),
                anchorArray(2,0), anchorArray(2,1), anchorArray(2,2),
                anchorArray(3,0), anchorArray(3,1), anchorArray(3,2));
        isAutoposition = true;
    }
    //MP0034,0,302,109,287,23,134.2,23.4,23,56
    else if((receive_buf[0] == 'M') && (receive_buf[1] == 'P'))
    {
        char *ptr, *retptr;
        ptr = (char*)receive_buf;
        char cut_data[30][12];
        int cut_count = 0;

        while((retptr = strtok(ptr,",")) != NULL )
        {
            //printf("%s\n", retptr);
            strcpy(cut_data[cut_count], retptr);
            ptr = NULL;
            cut_count++;
            if(cut_count >= 29)
                break;
        }
        
        int tag_id = atoi(cut_data[1]);
        printf("tag_id = %d\n", tag_id);

        float x = (float)atoi(cut_data[2]) / 100.0f;
        printf("x = %.2fm\n", x);
        
        float y = (float)atoi(cut_data[3]) / 100.0f;
        printf("y = %.2fm\n", y);   
        
        float aoa = atof(cut_data[7]);
        printf("aoa = %.2f°\n", aoa);
        
        float dis = (float)atoi(cut_data[4]) / 100.0f;
        printf("dis = %.2fm\n", dis);
    }
    else
    {
        puts("no range message");
        return;
    }
}


void CtrlSerDataDeal()
{
    unsigned char middata = 0;
    static unsigned char dataTmp[MAX_DATA_NUM] = {0};

    while(BufCtrlPosit_r != BufCtrlPosit_w)
    {
        middata = BufDataFromCtrl[BufCtrlPosit_r];
        BufCtrlPosit_r = (BufCtrlPosit_r==MAX_DATA_NUM-1)? 0 : (BufCtrlPosit_r+1);

        if(((middata == DataHead) || (middata == DataHead2))&&(rcvsign == 0))//收到头
        {
            rcvsign = 1;//开始了一个数据帧
            dataTmp[DataRecord++] = middata;//数据帧接收中
        }
        else if((middata != DataTail)&&(rcvsign == 1))
        {
            dataTmp[DataRecord++] = middata;//数据帧接收中
        }
        else if((middata == DataTail)&&(rcvsign == 1))//收到尾
        {
            if(DataRecord != 1)
            {
                rcvsign = 0;
                dataTmp[DataRecord++] = middata;
                dataTmp[DataRecord] = '\0';

                strncpy((char*)receive_buf, (char*)dataTmp, DataRecord);
                printf("receive_buf = %slen = %d\n", receive_buf, DataRecord);
                receive_deal_func(); /*调用处理函数*/
                bzero(receive_buf, sizeof(receive_buf));

                DataRecord = 0;
            }
        }
    }
}


int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
	std_msgs::String msg;
	std_msgs::String  msg_mc;
	int  data_size;
	int n;
	int cnt = 0;
    ros::init(argc, argv, "uwb_imu_node");//发布imu,uwb节点
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh;
    ros::NodeHandle nh1;
    ros::Publisher uwb_publisher = nh.advertise<Uwb_Location::uwb>("/uwb/data", 1000);//发布uwb数据  话题名 队列大小
    ros::Publisher IMU_read_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 1000);//发布imu话题

    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(11);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);

 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    
    //ros::Rate loop_rate(11);

    //发布uwb话题
    Uwb_Location::uwb uwb_data;
    //打包IMU数据
    sensor_msgs::Imu imu_data;

    

    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t len = sp.available();
        if(len > 0)
        {
            unsigned char usart_buf[1024]={0};
            sp.read(usart_buf, len);
            //printf("uart_data = %s\n", usart_buf);

            unsigned char *pbuf;
            unsigned char buf[2014] = {0};

            pbuf = (unsigned char *)usart_buf;
            memcpy(&buf[0], pbuf, len);

            int reallength = len;
            int i;
            if(reallength != 0)
            {
                for( i=0; i < reallength; i++)
                {
                    BufDataFromCtrl[BufCtrlPosit_w] = buf[i];

                    BufCtrlPosit_w = (BufCtrlPosit_w==(MAX_DATA_NUM-1))? 0 : (1 + BufCtrlPosit_w);
                }
            }
            CtrlSerDataDeal();
//---------------------------------UWB----------------------------------------------------
            uwb_data.time=ros::Time::now();
            uwb_data.x = report.x;
            uwb_data.y = report.y;
            uwb_data.z = report.z;
            //printf("tag.x=%.3f\r\ntag.y=%.3f\r\ntag.z=%.3f\r\n",uwb_data.x,uwb_data.y,uwb_data.z);

//--------------------------------------IMU------------------------------------------------
            //发布imu话题
            imu_data.header.stamp =uwb_data.time;
            imu_data.header.frame_id = "base_link";
            imu_data.linear_acceleration.x=velocityac[0];
            imu_data.linear_acceleration.y=velocityac[1];
            imu_data.linear_acceleration.z=velocityac[2];

            //角速度
            imu_data.angular_velocity.x = angleac[0]; 
            imu_data.angular_velocity.y = angleac[1]; 
            imu_data.angular_velocity.z = angleac[2];

            //四元数
            imu_data.orientation.x = Q.q1;
            imu_data.orientation.y = Q.q2;
            imu_data.orientation.z = Q.q3;
            imu_data.orientation.w = Q.q0;

//--------------------------------------话题发布------------------------------------
            uwb_publisher.publish(uwb_data);
            IMU_read_pub.publish(imu_data);
        }

        ros::spinOnce(); 
        //loop_rate.sleep();
    }
    //关闭串口
    sp.close();
    return 0;
}

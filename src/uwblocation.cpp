#include "Uwb_Location/uwblocation.h"

using namespace std;

unsigned char receive_buf[300] = {0};
vec3d report;
Eigen::MatrixXd anchorArray = Eigen::MatrixXd::Zero(8, 3);
Eigen::MatrixXd anchorArray_last = Eigen::MatrixXd::Zero(8, 3);
int result = 0; 
int tag_posi_success_cnt = 0; // 成功定位计数器
bool isAutoposition = false;
const bool AutopositionMode = false;

// 1：使用三边定位法；2：使用最小二乘法
const int tagposition_mode = 1;

string order_start = "$ancrangestart\r\n";
string order_stop = "$ancrangestop\r\n";

void receive_deal_func(serial::Serial& sp)
{
    // 初始化距离
    int range[8] = {-1};

    //  'mc' tag to anchor range bias corrected ranges – used for tag location
    if((receive_buf[0] == 'm') && (receive_buf[1] == 'c'))
    {
        if(AutopositionMode)
        {
            if(!isAutoposition)
            {   
                ROS_ERROR_STREAM("Using Autoposition Mode, but haven't autoposition yet!");
                // 发送串口指令'$ancrangestart\r\n'，打开基站自标定功能
                try {
                    sp.write(order_start);
                    cout << "order_start sent successfully!\n";
                } 
                catch (const std::exception& e) {
                    cerr << "Failed to send order_start: " << e.what() << "\n";
                }
                return;
            }
        }
        else
        {
            // 使用手动基站标定，则在下面更改标定坐标
            //A0 uint:m
            anchorArray(0,0) = 0.0; 
            anchorArray(0,1) = 0.0; 
            anchorArray(0,2) = 1.93; 
            //A1 uint:m
            anchorArray(1,0) = 4.25; 
            anchorArray(1,1) = 0.0; 
            anchorArray(1,2) = 1.93;
            //A2 uint:m
            anchorArray(2,0) = 4.3; 
            anchorArray(2,1) = 3.49; 
            anchorArray(2,2) = 1.93; 
            //A3 uint:m
            anchorArray(3,0) = 0.0; 
            anchorArray(3,1) = 3.49; 
            anchorArray(3,2) = 1.93; 
            //A4 uint:m
            anchorArray(4,0) = 2.0; 
            anchorArray(4,1) = 1.0; 
            anchorArray(4,2) = 2.5; 
            //A5 uint:m
            anchorArray(5,0) = 2.0; 
            anchorArray(5,1) = 0.0; 
            anchorArray(5,2) = 2.5;
            //A6 uint:m
            anchorArray(6,0) = 3.0; 
            anchorArray(6,1) = 1.0; 
            anchorArray(6,2) = 2.5; 
            //A7 uint:m
            anchorArray(7,0) = 3.0; 
            anchorArray(7,1) = 0.0; 
            anchorArray(7,2) = 2.5; 
        }

        int aid, tid, lnum, seq, mask;
        int rangetime;
        char role;
        int data_len = strlen((char*)receive_buf);
       
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

        result = GetLocation(&report, anchorArray, &range[0], tagposition_mode);

        printf("result = %d\n",result);
        printf("x = %f\n",report.x);
        printf("y = %f\n",report.y);
        printf("z = %f\n",report.z);

        if(result > 0)
        {
            tag_posi_success_cnt++;
        }  

        // tag定位成功5次后，则让上一次的基站自标定坐标失效，重新标定
        if(tag_posi_success_cnt == 5)
        {
            tag_posi_success_cnt = 0;
            isAutoposition = false; 
        }

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
        bool isSuccess = false;

        if((data_len == 106))
        {
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
                ROS_ERROR_STREAM("data Error !!!");
                return;
            }
        }
        else
        {
            ROS_ERROR_STREAM("data is not 106!");
            return;
        }

        // 获得上一次的标定后坐标
        anchorArray_last = anchorArray;

        // 对收集到的基站间距离进行自定位，获得定位后的坐标
        // 注意：该自定位算法只能定位各个基站的二维坐标(x,y)，z轴的坐标需要给定
        isSuccess = autopositioning(&range[0], aid, anchorArray);

        double dist = (anchorArray - anchorArray_last).rowwise().norm().sum();

        if(isSuccess && (dist < 0.05*4))
        {
            // 成功标定且上下两次标定的坐标偏差不超过0.1m
            isAutoposition = true;
            // 发送串口指令'$ancrangestop\r\n'，关闭基站自标定功能，开始标签定位
            printf("[Autoposition Success] A0:(%f, %f, %f),A1:(%f, %f, %f),A2:(%f, %f, %f),A3:(%f, %f, %f)\n",
                anchorArray(0,0), anchorArray(0,1), anchorArray(0,2),
                anchorArray(1,0), anchorArray(1,1), anchorArray(1,2),
                anchorArray(2,0), anchorArray(2,1), anchorArray(2,2),
                anchorArray(3,0), anchorArray(3,1), anchorArray(3,2));

            try {
                sp.write(order_stop);
                cout << "order_stop sent successfully!\n";
            } 
            catch (const std::exception& e) {
                cerr << "Failed to send order_stop: " << e.what() << "\n";
            }
            
            return;
        }



        return;
    }
    else
    {
        puts("No suitable message!\n");
        return;
    }
}


void CtrlSerDataDeal(serial::Serial& sp)
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
                receive_deal_func(sp); /*调用处理函数*/
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
	std_msgs::String msg_mc;
	int data_size;
	int n;
	int cnt = 0;
    ros::init(argc, argv, "uwb_location_node");//发布uwb节点
    //创建句柄
    ros::NodeHandle nh;
    ros::NodeHandle nh1;
    ros::Publisher uwb_publisher = nh.advertise<Uwb_Location::uwb>("/uwb/data", 1000);//发布tag的定位信息

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

    try{
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e){
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

    try {
        sp.write(order_start);
        std::cout << "Data sent successfully!\n";
    } 
    catch (const std::exception& e) {
        std::cerr << "Failed to send data: " << e.what() << "\n";
    }

    //发布uwb话题
    Uwb_Location::uwb uwb_data;

    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t len = sp.available();

        if(len > 0)
        {
            unsigned char usart_buf[1024]={0};
            sp.read(usart_buf, len);

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
            CtrlSerDataDeal(sp);
//---------------------------------UWB----------------------------------------------------
            uwb_data.time=ros::Time::now();
            uwb_data.x = report.x;
            uwb_data.y = report.y;
            uwb_data.z = report.z;
            //printf("tag.x=%.3f\r\ntag.y=%.3f\r\ntag.z=%.3f\r\n",uwb_data.x,uwb_data.y,uwb_data.z);
//--------------------------------------话题发布------------------------------------
            uwb_publisher.publish(uwb_data);
        }
        ros::spinOnce(); 
        //loop_rate.sleep();
    }
    //关闭串口
    sp.close();
    return 0;
}
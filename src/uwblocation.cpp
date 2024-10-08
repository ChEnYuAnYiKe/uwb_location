#include "uwb_location/uwblocation.h"

void Uwblocator::CtrlSerDataDeal(size_t len)
{
	unsigned char usart_buf[1024] = {0};
	sp.read(usart_buf, len);

	unsigned char *pbuf;
	unsigned char buf[2014] = {0};

	pbuf = (unsigned char *)usart_buf;
	memcpy(&buf[0], pbuf, len);

	int reallength = len;
	int i;
	if (reallength != 0)
	{
		for (i = 0; i < reallength; i++)
		{
			BufDataFromCtrl[BufCtrlPosit_w] = buf[i];

			BufCtrlPosit_w = (BufCtrlPosit_w == (MAX_DATA_NUM - 1))
									? 0
									: (1 + BufCtrlPosit_w);
		}
	}

	unsigned char middata = 0;
	static unsigned char dataTmp[MAX_DATA_NUM] = {0};

	while (BufCtrlPosit_r != BufCtrlPosit_w)
	{
		middata = BufDataFromCtrl[BufCtrlPosit_r];
		BufCtrlPosit_r =
			(BufCtrlPosit_r == MAX_DATA_NUM - 1) ? 0 : (BufCtrlPosit_r + 1);

		if (((middata == DataHead) || (middata == DataHead2)) &&
			(rcvsign == 0)) // 收到头
		{
			rcvsign = 1;					 // 开始了一个数据帧
			dataTmp[DataRecord++] = middata; // 数据帧接收中
		}
		else if ((middata != DataTail) && (rcvsign == 1))
		{
			dataTmp[DataRecord++] = middata; // 数据帧接收中
		}
		else if ((middata == DataTail) && (rcvsign == 1)) // 收到尾
		{
			if (DataRecord != 1)
			{
				rcvsign = 0;
				dataTmp[DataRecord++] = middata;
				dataTmp[DataRecord] = '\0';

				strncpy((char *)receive_buf, (char *)dataTmp, DataRecord);
				printf("receive_buf = %slen = %d\n", receive_buf, DataRecord);
				receive_deal_func(); /*调用处理函数*/
				bzero(receive_buf, sizeof(receive_buf));

				DataRecord = 0;
			}
		}
	}
}

void Uwblocator::receive_deal_func()
{
	//  'mc' tag to anchor range bias corrected ranges – used for tag location
	if ((receive_buf[0] == 'm') && (receive_buf[1] == 'c'))
	{
		// if (AnchorMode == 1)
		// {
		// 	if (!isAutoposition)
		// 	{
		// 		ROS_ERROR_STREAM(
		// 			"Using Autoposition Mode, but haven't autoposition yet!");
		// 		try
		// 		{
		// 			sp.write(order_start);
		// 			std::cout << "order_start sent successfully!\n";
		// 		}
		// 		catch (const std::exception &e)
		// 		{
		// 			std::cerr << "Failed to send order_start: " << e.what()
		// 					  << "\n";
		// 		}
		// 		return;
		// 	}
		// }
		// else if (AutopositionMode == 0)
		// {

		// }

		int aid, tid, lnum, seq, mask;
		int rangetime;
		char role;
		int data_len = strlen((char *)receive_buf);

		if (data_len == 106)
		{
			int n = sscanf((char *)receive_buf,
						   "mc %x %x %x %x %x %x %x %x %x %x %x %x %c%d:%d",
						   &mask, 
						   &trilaterator.range[0], &trilaterator.range[1], &trilaterator.range[2], &trilaterator.range[3],
						   &trilaterator.range[4], &trilaterator.range[5], &trilaterator.range[6], &trilaterator.range[7], 
						   &lnum, &seq, &rangetime, &role, &tid, &aid);

			printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d("
				   "mm)\nrange[3]=%d(mm)\nrange[4]=%d(mm)\nrange[5]=%d(mm)"
				   "\nrange[6]=%d(mm)\nrange[7]=%d(mm)\r\n",
				   mask, trilaterator.range[0], trilaterator.range[1], trilaterator.range[2], trilaterator.range[3], trilaterator.range[4], trilaterator.range[5], trilaterator.range[6], trilaterator.range[7]);
		}
		else if (data_len == 70)
		{
			int n = sscanf((char *)receive_buf, "mc %x %x %x %x %x %x %x %x %c%d:%d",
							&mask, 
							&trilaterator.range[0], &trilaterator.range[1], &trilaterator.range[2], &trilaterator.range[3], 
							&lnum, &seq, &rangetime, &role, &tid, &aid);

			printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d(mm)\nrange[3]=%d(mm)\r\n", mask, trilaterator.range[0], trilaterator.range[1], trilaterator.range[2], trilaterator.range[3]);
		}
		// else
		// {
		// 	return;
		// }
		
		// if (AnchorMode == 2)
		// {
		// 	boost::shared_lock<boost::shared_mutex> lock(anchorArrayMutex_);
		// 	result = trilaterator::GetLocation(&report, anchorArray, &range[0], TagpositionMode);
		// }
		// else if (AnchorMode == 0)
		// {
		// 	result = trilaterator::GetLocation(&report, anchorArray, &range[0], TagpositionMode);
		// }

		// printf("result = %d\n", result);
		// printf("x = %f\n", report.x);
		// printf("y = %f\n", report.y);
		// printf("z = %f\n", report.z);

		// return;
	}
	else
	{
		ROS_WARN_STREAM("Unknown data type!");
		// return;
	}

	return;
}

// if use Odometry
void Uwblocator::anchor1_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	nav_msgs::Odometry current_odom = *msg;
	trilaterator.anchorArray(0, 0) = current_odom.pose.pose.position.x;
	trilaterator.anchorArray(0, 1) = current_odom.pose.pose.position.y;
	trilaterator.anchorArray(0, 2) = current_odom.pose.pose.position.z;
}	
void Uwblocator::anchor2_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	nav_msgs::Odometry current_odom = *msg;
	trilaterator.anchorArray(1, 0) = current_odom.pose.pose.position.x;
	trilaterator.anchorArray(1, 1) = current_odom.pose.pose.position.y;
	trilaterator.anchorArray(1, 2) = current_odom.pose.pose.position.z;
}
void Uwblocator::anchor3_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	nav_msgs::Odometry current_odom = *msg;
	trilaterator.anchorArray(2, 0) = current_odom.pose.pose.position.x;
	trilaterator.anchorArray(2, 1) = current_odom.pose.pose.position.y;
	trilaterator.anchorArray(2, 2) = current_odom.pose.pose.position.z;
}
void Uwblocator::anchor4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	nav_msgs::Odometry current_odom = *msg;
	trilaterator.anchorArray(3, 0) = current_odom.pose.pose.position.x;
	trilaterator.anchorArray(3, 1) = current_odom.pose.pose.position.y;
	trilaterator.anchorArray(3, 2) = current_odom.pose.pose.position.z;
}

void Uwblocator::fix_anchor_callback(const ros::TimerEvent &event)
{
	result = trilaterator.GetLocation(&report, TagpositionMode);

	if (result == -1)
	{
		ROS_WARN_STREAM("Trilateration failed!");
		return;
	}

	uwb_data.header.stamp = ros::Time::now();
	uwb_data.pose.position.x = report.x;
	uwb_data.pose.position.y = report.y;
	uwb_data.pose.position.z = report.z;
	uwb_pub.publish(uwb_data);

	printf("x = %f, y = %f, z = %f", report.x, report.y, report.z);
}

void Uwblocator::dynamic_anchor_callback(const ros::TimerEvent &event)
{
	boost::shared_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	result = trilaterator.GetLocation(&report, TagpositionMode);

	if (result == -1)
	{
		ROS_WARN_STREAM("Trilateration failed!");
		return;
	}

	uwb_data.header.stamp = ros::Time::now();
	uwb_data.pose.position.x = report.x;
	uwb_data.pose.position.y = report.y;
	uwb_data.pose.position.z = report.z;
	uwb_pub.publish(uwb_data);

	printf("[uwb location] x = %f, y = %f, z = %f", report.x, report.y, report.z);
}

void Uwblocator::init(ros::NodeHandle &nh)
{
	// 初始化变量
	BufCtrlPosit_w = 0;
    BufCtrlPosit_r = 0;
	DataRecord = 0;
	rcvsign = 0;

	receive_buf[3000] = {0};
	result = 0;
	isAutoposition = false;

	order_start = "$ancrangestart\r\n";
	order_stop = "$ancrangestop\r\n";

	trilaterator.range[8] = {-1};

	height_extra = 0;

	uwb_pub = nh.advertise<geometry_msgs::PoseStamped>("/uwb/data", 500); // 发布tag的定位信息

	// Load configs.
	nh.param<int>("AnchorMode", AnchorMode, 0);
	nh.param<int>("TagpositionMode", TagpositionMode, 1);
	anchor1_pos_topic = nh.param<std::string>("anchor1_pos", "/robot_0/Odometry");
	anchor2_pos_topic = nh.param<std::string>("anchor2_pos", "/robot_1/Odometry");
	anchor3_pos_topic = nh.param<std::string>("anchor3_pos", "/robot_2/Odometry");
	anchor4_pos_topic = nh.param<std::string>("anchor4_pos", "/robot_3/Odometry");

	std::vector<double> point1, point2, point3, point4;
    nh.getParam("point1", point1);
    nh.getParam("point2", point2);
    nh.getParam("point3", point3);
    nh.getParam("point4", point4);

	nh.param("publish_rate", pub_rate, 200);
	pub_rate_inv = 1.0 / pub_rate;

	port_name = nh.param<std::string>("port_name", "/dev/usb_uwb");

	// 创建timeout
	serial::Timeout to = serial::Timeout::simpleTimeout(10);
	// 设置要打开的串口名称
	sp.setPort(port_name);
	// 设置串口通信的波特率
	sp.setBaudrate(115200);
	// 串口设置timeout
	sp.setTimeout(to);

	try
	{
		sp.open();
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR_STREAM("Unable to open usb port.");
		return;
	}
	// 判断串口是否打开成功
	if (sp.isOpen())
		ROS_INFO_STREAM("Usb port is opened.");
	else
		return;

	if (AnchorMode == 0)
	{
		// 手动基站标定，若基站位置修改则在下面更改标定坐标 uint:m
		trilaterator.anchorArray << point1[0], point1[1], point1[2], 
									point2[0], point2[1], point2[2], 
									point3[0], point3[1], point3[2], 
									point4[0], point4[1], point4[2];

		compute_pos_timer = nh.createTimer(ros::Duration(pub_rate_inv), &Uwblocator::fix_anchor_callback, this, false, false);

		ROS_WARN_STREAM("AutopositionMode == 0! Using fixed anchor position!");
		ros::Duration(0.7).sleep();
	}
	else if (AnchorMode == 2)
	{
		anchor1_pos_sub = nh.subscribe(anchor1_pos_topic, 500, &Uwblocator::anchor1_odom_callback, this);
		anchor2_pos_sub = nh.subscribe(anchor2_pos_topic, 500, &Uwblocator::anchor2_odom_callback, this);
		anchor3_pos_sub = nh.subscribe(anchor3_pos_topic, 500, &Uwblocator::anchor3_odom_callback, this);
		anchor4_pos_sub = nh.subscribe(anchor4_pos_topic, 500, &Uwblocator::anchor4_odom_callback, this);

		compute_pos_timer = nh.createTimer(ros::Duration(pub_rate_inv), &Uwblocator::dynamic_anchor_callback, this, false, false);

		ROS_WARN_STREAM("AutopositionMode == 2! Subsribed to 4 anchors' odom topic!");
		ros::Duration(0.7).sleep();
	}

	return;
}

void Uwblocator::run()
{
	compute_pos_timer.start();

	while (ros::ok())
	{
		ros::spinOnce();
		// 获取缓冲区内的字节数
		size_t len_ = sp.available();

		if (len_ > 0)
		{
			CtrlSerDataDeal(len_);
		}
	}
	// 关闭串口
	sp.close();
	return;
}


// int main(int argc, char **argv)
// {
// 	setlocale(LC_ALL, "");
// 	std_msgs::String msg;
// 	std_msgs::String msg_mc;
// 	int data_size;
// 	int n;
// 	int cnt = 0;
// 	ros::init(argc, argv, "uwb_location_node"); // 发布uwb节点
// 	// 创建句柄
// 	ros::NodeHandle nh;
// 	ros::NodeHandle nh1;
// 	// ros::Publisher uwb_publisher = nh.advertise<uwb_location::uwb>("/uwb/data", 500); // 发布tag的定位信息
// 	ros::Publisher uwb_publisher = nh.advertise<geometry_msgs::PoseStamped>("/uwb/data", 500); // 发布tag的定位信息

// 	// 创建一个serial类
// 	serial::Serial sp;
// 	// 创建timeout
// 	serial::Timeout to = serial::Timeout::simpleTimeout(11);
// 	// 设置要打开的串口名称
// 	sp.setPort("/dev/usb_uwb");
// 	// 设置串口通信的波特率
// 	sp.setBaudrate(115200);
// 	// 串口设置timeout
// 	sp.setTimeout(to);

// 	// Load configs.
// 	nh.param("AutopositionMode", AutopositionMode, 2);
// 	nh.param("TagpositionMode", TagpositionMode, 2);

// 	nh.param("anchor1_pos", anchor1_pos_topic);
// 	nh.param("anchor2_pos", anchor2_pos_topic);
// 	nh.param("anchor3_pos", anchor3_pos_topic);
// 	nh.param("anchor4_pos", anchor4_pos_topic);

// 	try
// 	{
// 		// 打开串口
// 		sp.open();
// 	}
// 	catch (serial::IOException &e)
// 	{
// 		ROS_ERROR_STREAM("Unable to open port.");
// 		return -1;
// 	}

// 	// 判断串口是否打开成功
// 	if (sp.isOpen())
// 	{
// 		ROS_INFO_STREAM("/dev/usb_uwb is opened.");
// 	}
// 	else
// 	{
// 		return -1;
// 	}

// 	// ros::Rate loop_rate(200);

// 	if (AutopositionMode == 1)
// 	{
// 		std::cout << "AutopositionMode == 1!" << std::endl;
// 		try
// 		{
// 			sp.write(order_start);
// 			std::cout << "order_start sent successfully!\n";
// 		}
// 		catch (const std::exception &e)
// 		{
// 			std::cerr << "Failed to send data: " << e.what() << "\n";
// 		}
// 	}
// 	else if (AutopositionMode == 2)
// 	{
// 		// anchor1_pos_sub =
// 		// 	nh.subscribe(anchor1_pos_topic, 100, anchor1_pos_callback);
// 		// anchor2_pos_sub =
// 		// 	nh.subscribe(anchor2_pos_topic, 100, anchor2_pos_callback);
// 		// anchor3_pos_sub =
// 		// 	nh.subscribe(anchor3_pos_topic, 100, anchor3_pos_callback);
// 		// anchor4_pos_sub =
// 		// 	nh.subscribe(anchor4_pos_topic, 100, anchor4_pos_callback);

// 		anchor1_pos_sub = nh.subscribe(anchor1_pos_topic, 100, anchor1_odom_callback);
// 		anchor2_pos_sub = nh.subscribe(anchor2_pos_topic, 100, anchor2_odom_callback);
// 		anchor3_pos_sub = nh.subscribe(anchor3_pos_topic, 100, anchor3_odom_callback);
// 		anchor4_pos_sub = nh.subscribe(anchor4_pos_topic, 100, anchor4_odom_callback);

// 		ROS_WARN_STREAM("AutopositionMode == 2! Subsribed to 4 anchors' positon topic!");
// 		ros::Duration(1).sleep();
// 	}
// 	else if (AutopositionMode == 0)
// 	{
// 		// 手动基站标定，若基站位置修改则在下面更改标定坐标
// 		// A0 uint:m
// 		anchorArray(0, 0) = 0.0;
// 		anchorArray(0, 1) = 0.0;
// 		anchorArray(0, 2) = 0.2;
// 		// A1 uint:m
// 		anchorArray(1, 0) = 6.54;
// 		anchorArray(1, 1) = 0.0;
// 		anchorArray(1, 2) = 0.2;
// 		// A2 uint:m
// 		anchorArray(2, 0) = 6.71;
// 		anchorArray(2, 1) = 6.80;
// 		anchorArray(2, 2) = 0.2;
// 		// A3 uint:m
// 		anchorArray(3, 0) = -0.06;
// 		anchorArray(3, 1) = 6.72;
// 		anchorArray(3, 2) = 0.2;

// 		// A4 uint:m
// 		anchorArray(4, 0) = 2.0;
// 		anchorArray(4, 1) = 1.0;
// 		anchorArray(4, 2) = 2.5;
// 		// A5 uint:m
// 		anchorArray(5, 0) = 2.0;
// 		anchorArray(5, 1) = 0.0;
// 		anchorArray(5, 2) = 2.5;
// 		// A6 uint:m
// 		anchorArray(6, 0) = 3.0;
// 		anchorArray(6, 1) = 1.0;
// 		anchorArray(6, 2) = 2.5;
// 		// A7 uint:m
// 		anchorArray(7, 0) = 3.0;
// 		anchorArray(7, 1) = 0.0;
// 		anchorArray(7, 2) = 2.5;
// 		ROS_WARN_STREAM("AutopositionMode == 0! Using fixed anchor position!");
// 		ros::Duration(1).sleep();
// 	}

// 	// 发布uwb话题
// 	// uwb_location::uwb uwb_data;
// 	geometry_msgs::PoseStamped uwb_data;

// 	while (ros::ok())
// 	{
// 		ros::spinOnce();
// 		// 获取缓冲区内的字节数
// 		size_t len = sp.available();

// 		if (len > 0)
// 		{
// 			unsigned char usart_buf[1024] = {0};
// 			sp.read(usart_buf, len);

// 			unsigned char *pbuf;
// 			unsigned char buf[2014] = {0};

// 			pbuf = (unsigned char *)usart_buf;
// 			memcpy(&buf[0], pbuf, len);

// 			int reallength = len;
// 			int i;
// 			if (reallength != 0)
// 			{
// 				for (i = 0; i < reallength; i++)
// 				{
// 					BufDataFromCtrl[BufCtrlPosit_w] = buf[i];

// 					BufCtrlPosit_w = (BufCtrlPosit_w == (MAX_DATA_NUM - 1))
// 										 ? 0
// 										 : (1 + BufCtrlPosit_w);
// 				}
// 			}
// 			CtrlSerDataDeal(sp);
// 			//---------------------------------UWB----------------------------------------------------
// 			uwb_data.header.stamp = ros::Time::now();
// 			uwb_data.pose.position.x = report.x;
// 			uwb_data.pose.position.y = report.y;
// 			uwb_data.pose.position.z = report.z;
// 			// printf("tag.x=%.3f\r\ntag.y=%.3f\r\ntag.z=%.3f\r\n",uwb_data.x,uwb_data.y,uwb_data.z);
// 			//--------------------------------------话题发布------------------------------------
// 			uwb_publisher.publish(uwb_data);
// 		}
// 		// loop_rate.sleep();
// 	}
// 	// 关闭串口
// 	sp.close();
// 	return 0;
// }
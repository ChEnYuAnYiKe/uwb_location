#include "uwb_location/uwblocation.h"

unsigned char receive_buf[3000] = {0};

int result = 0;
bool isAutoposition = false;

void receive_deal_func(serial::Serial &sp)
{
	// 初始化距离
	int range[8] = {-1};

	//  'mc' tag to anchor range bias corrected ranges – used for tag location
	if ((receive_buf[0] == 'm') && (receive_buf[1] == 'c'))
	{
		if (AutopositionMode == 1)
		{
			if (!isAutoposition)
			{
				ROS_ERROR_STREAM(
					"Using Autoposition Mode, but haven't autoposition yet!");
				try
				{
					sp.write(order_start);
					std::cout << "order_start sent successfully!\n";
				}
				catch (const std::exception &e)
				{
					std::cerr << "Failed to send order_start: " << e.what()
							  << "\n";
				}
				return;
			}
		}
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
						   &mask, &range[0], &range[1], &range[2], &range[3],
						   &range[4], &range[5], &range[6], &range[7], &lnum,
						   &seq, &rangetime, &role, &tid, &aid);
			printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d("
				   "mm)\nrange[3]=%d(mm)\nrange[4]=%d(mm)\nrange[5]=%d(mm)"
				   "\nrange[6]=%d(mm)\nrange[7]=%d(mm)\r\n",
				   mask, range[0], range[1], range[2], range[3], range[4],
				   range[5], range[6], range[7]);
		}
		else if (data_len == 70)
		{
			int n =
				sscanf((char *)receive_buf, "mc %x %x %x %x %x %x %x %x %c%d:%d",
					   &mask, &range[0], &range[1], &range[2], &range[3], &lnum,
					   &seq, &rangetime, &role, &tid, &aid);
			printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d("
				   "mm)\nrange[3]=%d(mm)\r\n",
				   mask, range[0], range[1], range[2], range[3]);
		}
		else
		{
			return;
		}
		if (AutopositionMode == 2)
		{
			boost::shared_lock<boost::shared_mutex> lock(anchorArrayMutex_);
			result = GetLocation(&report, anchorArray, &range[0], TagpositionMode);
		}
		else
		{
			result = GetLocation(&report, anchorArray, &range[0], TagpositionMode);
		}

		printf("result = %d\n", result);
		printf("x = %f\n", report.x);
		printf("y = %f\n", report.y);
		printf("z = %f\n", report.z);

		return;
	}
	// 'ma' anchor to anchor range bias corrected ranges – used for anchor
	// auto-positioning
	else if ((receive_buf[0] == 'm') && (receive_buf[1] == 'a'))
	{
		int aid, tid, lnum, seq, mask;
		int rangetime;
		char role;
		int rx_power;
		int data_len = strlen((char *)receive_buf);
		bool isSuccess = false;

		if ((data_len == 106))
		{
			int n = sscanf((char *)receive_buf,
						   "ma %x %x %x %x %x %x %x %x %x %x %x %x %c%d:%d %x",
						   &mask, &range[0], &range[1], &range[2], &range[3],
						   &range[4], &range[5], &range[6], &range[7], &lnum,
						   &seq, &rangetime, &role, &tid, &aid, &rx_power);
			printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d("
				   "mm)\nrange[3]=%d(mm)\nrole=%c:%d\r\n",
				   mask, range[0], range[1], range[2], range[3], role, aid);

			if (n == EOF)
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

		if (isSuccess && (dist < 0.05 * 4))
		{
			// 成功标定且上下两次标定的坐标偏差不超过0.05m
			isAutoposition = true;
			// 发送串口指令'$ancrangestop\r\n'，关闭基站自标定功能，开始标签定位
			ROS_WARN_STREAM("Self-calibration Succeed!");
			printf("A0:(%f, %f, %f),A1:(%f, %f, %f),A2:(%f, %f, %f),A3:(%f, "
				   "%f, %f)\n",
				   anchorArray(0, 0), anchorArray(0, 1), anchorArray(0, 2),
				   anchorArray(1, 0), anchorArray(1, 1), anchorArray(1, 2),
				   anchorArray(2, 0), anchorArray(2, 1), anchorArray(2, 2),
				   anchorArray(3, 0), anchorArray(3, 1), anchorArray(3, 2));

			try
			{
				sp.write(order_stop);
				std::cout << "order_stop sent successfully!\n";
			}
			catch (const std::exception &e)
			{
				std::cerr << "Failed to send order_stop: " << e.what() << "\n";
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

void CtrlSerDataDeal(serial::Serial &sp)
{
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
				receive_deal_func(sp); /*调用处理函数*/
				bzero(receive_buf, sizeof(receive_buf));

				DataRecord = 0;
			}
		}
	}
}

void anchor1_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	geometry_msgs::PoseStamped current_vrpn = *msg;
	anchorArray(0, 0) = current_vrpn.pose.position.x;
	anchorArray(0, 1) = current_vrpn.pose.position.y;
	anchorArray(0, 2) = current_vrpn.pose.position.z + 0.15;
}

void anchor2_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	geometry_msgs::PoseStamped current_vrpn = *msg;
	anchorArray(1, 0) = current_vrpn.pose.position.x;
	anchorArray(1, 1) = current_vrpn.pose.position.y;
	anchorArray(1, 2) = current_vrpn.pose.position.z + 0.15;
}

void anchor3_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	geometry_msgs::PoseStamped current_vrpn = *msg;
	anchorArray(2, 0) = current_vrpn.pose.position.x;
	anchorArray(2, 1) = current_vrpn.pose.position.y;
	anchorArray(2, 2) = current_vrpn.pose.position.z + 0.15;
}

void anchor4_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	geometry_msgs::PoseStamped current_vrpn = *msg;
	anchorArray(3, 0) = current_vrpn.pose.position.x;
	anchorArray(3, 1) = current_vrpn.pose.position.y;
	anchorArray(3, 2) = current_vrpn.pose.position.z + 0.15;
}

// if use fast-lio
void anchor1_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	nav_msgs::Odometry current_odom = *msg;
	anchorArray(0, 0) = current_odom.pose.pose.position.x;
	anchorArray(0, 1) = current_odom.pose.pose.position.y;
	anchorArray(0, 2) = current_odom.pose.pose.position.z;
}	

void anchor2_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	nav_msgs::Odometry current_odom = *msg;
	anchorArray(1, 0) = current_odom.pose.pose.position.x;
	anchorArray(1, 1) = current_odom.pose.pose.position.y;
	anchorArray(1, 2) = current_odom.pose.pose.position.z;
}

void anchor3_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	nav_msgs::Odometry current_odom = *msg;
	anchorArray(2, 0) = current_odom.pose.pose.position.x;
	anchorArray(2, 1) = current_odom.pose.pose.position.y;
	anchorArray(2, 2) = current_odom.pose.pose.position.z;
}

void anchor4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_);
	nav_msgs::Odometry current_odom = *msg;
	anchorArray(3, 0) = current_odom.pose.pose.position.x;
	anchorArray(3, 1) = current_odom.pose.pose.position.y;
	anchorArray(3, 2) = current_odom.pose.pose.position.z;
}

int main(int argc, char **argv)
{
	setlocale(LC_ALL, "");
	std_msgs::String msg;
	std_msgs::String msg_mc;
	int data_size;
	int n;
	int cnt = 0;
	ros::init(argc, argv, "uwb_location_node"); // 发布uwb节点
	// 创建句柄
	ros::NodeHandle nh;
	ros::NodeHandle nh1;
	// ros::Publisher uwb_publisher = nh.advertise<uwb_location::uwb>("/uwb/data", 500); // 发布tag的定位信息
	ros::Publisher uwb_publisher = nh.advertise<geometry_msgs::PoseStamped>("/uwb/data", 500); // 发布tag的定位信息

	// 创建一个serial类
	serial::Serial sp;
	// 创建timeout
	serial::Timeout to = serial::Timeout::simpleTimeout(11);
	// 设置要打开的串口名称
	sp.setPort("/dev/usb_uwb");
	// 设置串口通信的波特率
	sp.setBaudrate(115200);
	// 串口设置timeout
	sp.setTimeout(to);

	// Load configs.
	nh.param("AutopositionMode", AutopositionMode, 0);
	nh.param("TagpositionMode", TagpositionMode, 1);

	// nh.param("anchor1_pos", anchor1_pos_topic);
	// nh.param("anchor2_pos", anchor2_pos_topic);
	// nh.param("anchor3_pos", anchor3_pos_topic);
	// nh.param("anchor4_pos", anchor4_pos_topic);

	try
	{
		// 打开串口
		sp.open();
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR_STREAM("Unable to open port.");
		return -1;
	}

	// 判断串口是否打开成功
	if (sp.isOpen())
	{
		ROS_INFO_STREAM("/dev/usb_uwb is opened.");
	}
	else
	{
		return -1;
	}

	ros::Rate loop_rate(200);

	if (AutopositionMode == 1)
	{
		std::cout << "AutopositionMode == 1!" << std::endl;
		try
		{
			sp.write(order_start);
			std::cout << "order_start sent successfully!\n";
		}
		catch (const std::exception &e)
		{
			std::cerr << "Failed to send data: " << e.what() << "\n";
		}
	}
	else if (AutopositionMode == 2)
	{
		// anchor1_pos_sub =
		// 	nh.subscribe(anchor1_pos_topic, 100, anchor1_pos_callback);
		// anchor2_pos_sub =
		// 	nh.subscribe(anchor2_pos_topic, 100, anchor2_pos_callback);
		// anchor3_pos_sub =
		// 	nh.subscribe(anchor3_pos_topic, 100, anchor3_pos_callback);
		// anchor4_pos_sub =
		// 	nh.subscribe(anchor4_pos_topic, 100, anchor4_pos_callback);

		anchor1_pos_sub = nh.subscribe(anchor1_pos_topic, 100, anchor1_odom_callback);
		anchor2_pos_sub = nh.subscribe(anchor2_pos_topic, 100, anchor2_odom_callback);
		anchor3_pos_sub = nh.subscribe(anchor3_pos_topic, 100, anchor3_odom_callback);
		anchor4_pos_sub = nh.subscribe(anchor4_pos_topic, 100, anchor4_odom_callback);

		ROS_WARN_STREAM("AutopositionMode == 2! Subsribed to 4 anchors' positon topic!");
		ros::Duration(1).sleep();
	}
	else if (AutopositionMode == 0)
	{
		// 手动基站标定，若基站位置修改则在下面更改标定坐标
		// A0 uint:m
		anchorArray(0, 0) = 0.0;
		anchorArray(0, 1) = 0.0;
		anchorArray(0, 2) = 0.2;
		// A1 uint:m
		anchorArray(1, 0) = 6.54;
		anchorArray(1, 1) = 0.0;
		anchorArray(1, 2) = 0.2;
		// A2 uint:m
		anchorArray(2, 0) = 6.71;
		anchorArray(2, 1) = 6.80;
		anchorArray(2, 2) = 0.2;
		// A3 uint:m
		anchorArray(3, 0) = -0.06;
		anchorArray(3, 1) = 6.72;
		anchorArray(3, 2) = 0.2;

		// A4 uint:m
		anchorArray(4, 0) = 2.0;
		anchorArray(4, 1) = 1.0;
		anchorArray(4, 2) = 2.5;
		// A5 uint:m
		anchorArray(5, 0) = 2.0;
		anchorArray(5, 1) = 0.0;
		anchorArray(5, 2) = 2.5;
		// A6 uint:m
		anchorArray(6, 0) = 3.0;
		anchorArray(6, 1) = 1.0;
		anchorArray(6, 2) = 2.5;
		// A7 uint:m
		anchorArray(7, 0) = 3.0;
		anchorArray(7, 1) = 0.0;
		anchorArray(7, 2) = 2.5;
		ROS_WARN_STREAM("AutopositionMode == 0! Using fixed anchor position!");
		ros::Duration(1).sleep();
	}

	// 发布uwb话题
	// uwb_location::uwb uwb_data;
	geometry_msgs::PoseStamped uwb_data;

	while (ros::ok())
	{
		ros::spinOnce();
		// 获取缓冲区内的字节数
		size_t len = sp.available();

		if (len > 0)
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
			CtrlSerDataDeal(sp);
			//---------------------------------UWB----------------------------------------------------
			uwb_data.header.stamp = ros::Time::now();
			uwb_data.pose.position.x = report.x;
			uwb_data.pose.position.y = report.y;
			uwb_data.pose.position.z = report.z;
			// printf("tag.x=%.3f\r\ntag.y=%.3f\r\ntag.z=%.3f\r\n",uwb_data.x,uwb_data.y,uwb_data.z);
			//--------------------------------------话题发布------------------------------------
			uwb_publisher.publish(uwb_data);
		}
		loop_rate.sleep();
	}
	// 关闭串口
	sp.close();
	return 0;
}
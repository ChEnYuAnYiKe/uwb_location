#include "uwb_location/uwblocation.h"

uwbPositionSystem::uwbPositionSystem(ros::NodeHandle &nh, serial::Serial& sp)
{
	isAutoposition = false;
	result = 0;
	order_start = "$ancrangestart\r\n";
	order_stop = "$ancrangestop\r\n";

	receive_buf[3000] = {0};
	BufCtrlPosit_w = 0;
	BufCtrlPosit_r = 0;

	DataRecord = 0;
	rcvsign = 0;

	anchorArray = Eigen::MatrixXd::Zero(8, 3);
	anchorArray_last = Eigen::MatrixXd::Zero(8, 3);

	// Load configs.
	nh.param("AutopositionMode", AutopositionMode, 0);
	nh.param("TagpositionMode", TagpositionMode, 1);
	nh.param("anchor1_pos", anchor1_pos_topic);
	nh.param("anchor2_pos", anchor2_pos_topic);
	nh.param("anchor3_pos", anchor3_pos_topic);
	nh.param("anchor4_pos", anchor4_pos_topic);

	uwb_publisher = nh.advertise<uwb_location::uwb>("/uwb/data", 100); // 发布tag的定位信息

	if (AutopositionMode == 0)
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
		anchorArray(4, 0) = 0;
		anchorArray(4, 1) = 0;
		anchorArray(4, 2) = 0;
		// A5 uint:m
		anchorArray(5, 0) = 0;
		anchorArray(5, 1) = 0;
		anchorArray(5, 2) = 0;
		// A6 uint:m
		anchorArray(6, 0) = 0;
		anchorArray(6, 1) = 0;
		anchorArray(6, 2) = 0;
		// A7 uint:m
		anchorArray(7, 0) = 0;
		anchorArray(7, 1) = 0;
		anchorArray(7, 2) = 0;

		ROS_INFO_STREAM("AutopositionMode == 0! Using fixed anchor position!");

		ros::Duration(0.5).sleep();
	}
	else if (AutopositionMode == 1)
	{
		ROS_INFO_STREAM("AutopositionMode == 1!");
		try
		{
			sp.write(order_start);
			ROS_INFO_STREAM("order_start sent successfully!");
		}
		catch (const std::exception &e)
		{
			std::cerr << "Failed to send data: " << e.what() << "\n";
		}
	}
	else if (AutopositionMode == 2)
	{
		anchor1_pos_sub =
			nh.subscribe(anchor1_pos_topic, 100, &uwbPositionSystem::anchor1_pos_callback, this);
		anchor2_pos_sub =
			nh.subscribe(anchor2_pos_topic, 100, &uwbPositionSystem::anchor2_pos_callback, this);
		anchor3_pos_sub =
			nh.subscribe(anchor3_pos_topic, 100, &uwbPositionSystem::anchor3_pos_callback, this);
		anchor4_pos_sub =
			nh.subscribe(anchor4_pos_topic, 100, &uwbPositionSystem::anchor4_pos_callback, this);

		// debug
		array_row1_pub = nh.advertise<geometry_msgs::PoseStamped>("/anchorArray/row1", 10);
		array_row2_pub = nh.advertise<geometry_msgs::PoseStamped>("/anchorArray/row2", 10);
		array_row3_pub = nh.advertise<geometry_msgs::PoseStamped>("/anchorArray/row3", 10);
		array_row4_pub = nh.advertise<geometry_msgs::PoseStamped>("/anchorArray/row4", 10);

		ROS_INFO_STREAM("AutopositionMode == 2! Subscribed to 4 anchors' positon topic!");

		ros::Duration(0.5).sleep();
	} else
	{
		ROS_WARN_STREAM("Undefined AutopositionMode!");
	}
}

uwbPositionSystem::~uwbPositionSystem(){
}


void uwbPositionSystem::receive_deal_func(serial::Serial &sp)
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
			boost::shared_lock<boost::shared_mutex> lock(anchorArrayMutex_); // 加读锁
			
			result = GetLocation(&report, anchorArray, &range[0], TagpositionMode);
			uwbPositionSystem::array_pub_wrapper(anchorArray);
		}
		else
		{
			result = GetLocation(&report, anchorArray, &range[0], TagpositionMode);
		}

		printf("result = %d\n", result);
		printf("x = %f ", report.x);
		printf("y = %f ", report.y);
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

void uwbPositionSystem::CtrlSerDataDeal(serial::Serial &sp)
{
	unsigned char middata = 0;
	static unsigned char dataTmp[MAX_DATA_NUM] = {0};

	while (BufCtrlPosit_r != BufCtrlPosit_w)
	{
		middata = BufDataFromCtrl[BufCtrlPosit_r];
		BufCtrlPosit_r =
			(BufCtrlPosit_r == MAX_DATA_NUM - 1) ? 0 : (BufCtrlPosit_r + 1);

		if (((middata == DataHead) || (middata == DataHead2)) && (rcvsign == 0)) // 收到头
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

void uwbPositionSystem::processSerialData(serial::Serial& sp)
{
	size_t len = sp.available();
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
									? 0 : (1 + BufCtrlPosit_w);
		}
	}
}

void uwbPositionSystem::anchor1_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_); // 加写锁
	geometry_msgs::PoseStamped current_vrpn_1 = *msg;
	anchorArray(0, 0) = current_vrpn_1.pose.position.x;
	anchorArray(0, 1) = current_vrpn_1.pose.position.y;
	anchorArray(0, 2) = current_vrpn_1.pose.position.z;
}

void uwbPositionSystem::anchor2_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_); // 加写锁
	geometry_msgs::PoseStamped current_vrpn_2 = *msg;
	anchorArray(1, 0) = current_vrpn_2.pose.position.x;
	anchorArray(1, 1) = current_vrpn_2.pose.position.y;
	anchorArray(1, 2) = current_vrpn_2.pose.position.z;
}

void uwbPositionSystem::anchor3_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_); // 加写锁
	geometry_msgs::PoseStamped current_vrpn_3 = *msg;
	anchorArray(2, 0) = current_vrpn_3.pose.position.x;
	anchorArray(2, 1) = current_vrpn_3.pose.position.y;
	anchorArray(2, 2) = current_vrpn_3.pose.position.z;
}

void uwbPositionSystem::anchor4_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	boost::unique_lock<boost::shared_mutex> lock(anchorArrayMutex_); // 加写锁
	geometry_msgs::PoseStamped current_vrpn_4 = *msg;
	anchorArray(3, 0) = current_vrpn_4.pose.position.x;
	anchorArray(3, 1) = current_vrpn_4.pose.position.y;
	anchorArray(3, 2) = current_vrpn_4.pose.position.z;
}

void uwbPositionSystem::array_pub_wrapper(Eigen::MatrixXd& anchorArray)
{
	array_row1.header.frame_id = "global";
	array_row1.header.stamp = ros::Time::now();
	array_row1.pose.position.x = anchorArray(0, 0);
	array_row1.pose.position.y = anchorArray(0, 1);
	array_row1.pose.position.z = anchorArray(0, 2);
	array_row1_pub.publish(array_row1);

	array_row2.header.frame_id = "global";
	array_row2.header.stamp = ros::Time::now();
	array_row2.pose.position.x = anchorArray(1, 0);
	array_row2.pose.position.y = anchorArray(1, 1);
	array_row2.pose.position.z = anchorArray(1, 2);
	array_row2_pub.publish(array_row2);

	array_row3.header.frame_id = "global";
	array_row3.header.stamp = ros::Time::now();
	array_row3.pose.position.x = anchorArray(2, 0);
	array_row3.pose.position.y = anchorArray(2, 1);
	array_row3.pose.position.z = anchorArray(2, 2);
	array_row3_pub.publish(array_row3);

	array_row4.header.frame_id = "global";
	array_row4.header.stamp = ros::Time::now();
	array_row4.pose.position.x = anchorArray(3, 0);
	array_row4.pose.position.y = anchorArray(3, 1);
	array_row4.pose.position.z = anchorArray(3, 2);
	array_row4_pub.publish(array_row4);
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
		ROS_INFO_STREAM("/dev/usb_uwb port is opened.");
	}
	else
	{
		return -1;
	}

	// ros::Rate loop_rate(11);

	uwbPositionSystem uwb_sys(nh, sp);

	// uwb话题
	uwb_location::uwb uwb_data;

	while (ros::ok())
	{
		ros::spinOnce();
		// 获取缓冲区内的字节数
		size_t len = sp.available();

		if (len > 0)
		{
			uwb_sys.processSerialData(sp);
			uwb_sys.CtrlSerDataDeal(sp);
			//---------------------------------UWB----------------------------------------------------
			uwb_data.time = ros::Time::now();
			uwb_data.x = uwb_sys.report.x;
			uwb_data.y = uwb_sys.report.y;
			uwb_data.z = uwb_sys.report.z;
			// printf("tag.x=%.3f\r\ntag.y=%.3f\r\ntag.z=%.3f\r\n",uwb_data.x,uwb_data.y,uwb_data.z);
			//--------------------------------------话题发布------------------------------------
			uwb_sys.uwb_publisher.publish(uwb_data);
		}
		else
		{
			ROS_WARN_STREAM("No data received!");
		}
		// loop_rate.sleep();
	}
	// 关闭串口
	sp.close();
	return 0;
}
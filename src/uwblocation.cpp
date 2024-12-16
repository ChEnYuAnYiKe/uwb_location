#include "uwb_location/uwblocation.h"

unsigned char receive_buf[3000] = {0};

int range[8] = {-1};

void receive_deal_func(serial::Serial& sp) {
	//  'mc' tag to anchor range bias corrected ranges – used for tag location
	if ((receive_buf[0] == 'm') && (receive_buf[1] == 'c')) {
		int aid, tid, lnum, seq, mask;
		int rangetime;
		char role;
		int data_len = strlen((char*)receive_buf);

		if (data_len == 106) {
			sscanf((char*)receive_buf,
			       "mc %x %x %x %x %x %x %x %x %x %x %x %x %c%d:%d", &mask,
			       &range[0], &range[1], &range[2], &range[3], &range[4],
			       &range[5], &range[6], &range[7], &lnum, &seq, &rangetime,
			       &role, &tid, &aid);
			// printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d("
			// 	   "mm)\nrange[3]=%d(mm)\nrange[4]=%d(mm)\nrange[5]=%d(mm)"
			// 	   "\nrange[6]=%d(mm)\nrange[7]=%d(mm)\r\n",
			// 	   mask, range[0], range[1], range[2], range[3], range[4],
			// 	   range[5], range[6], range[7]);
		} else if (data_len == 70) {
			sscanf((char*)receive_buf, "mc %x %x %x %x %x %x %x %x %c%d:%d",
			       &mask, &range[0], &range[1], &range[2], &range[3], &lnum,
			       &seq, &rangetime, &role, &tid, &aid);
			// printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d("
			// 	   "mm)\nrange[3]=%d(mm)\r\n",
			// 	   mask, range[0], range[1], range[2], range[3]);
		}
		return;
	} else {
		puts("No suitable message!\n");
		return;
	}
}

void CtrlSerDataDeal(serial::Serial& sp, ros::Publisher& pub) {
	unsigned char middata = 0;
	static unsigned char dataTmp[MAX_DATA_NUM] = {0};

	while (BufCtrlPosit_r != BufCtrlPosit_w) {
		middata = BufDataFromCtrl[BufCtrlPosit_r];
		BufCtrlPosit_r =
		    (BufCtrlPosit_r == MAX_DATA_NUM - 1) ? 0 : (BufCtrlPosit_r + 1);

		if (((middata == DataHead) || (middata == DataHead2)) &&
		    (rcvsign == 0)) // 收到头
		{
			rcvsign = 1;                     // 开始了一个数据帧
			dataTmp[DataRecord++] = middata; // 数据帧接收中
		} else if ((middata != DataTail) && (rcvsign == 1)) {
			dataTmp[DataRecord++] = middata; // 数据帧接收中
		} else if ((middata == DataTail) && (rcvsign == 1)) // 收到尾
		{
			if (DataRecord != 1) {
				rcvsign = 0;
				dataTmp[DataRecord++] = middata;
				dataTmp[DataRecord] = '\0';

				strncpy((char*)receive_buf, (char*)dataTmp, DataRecord);
				// printf("receive_buf = %slen = %d\n", receive_buf,
				// DataRecord);
				receive_deal_func(sp); /*调用处理函数*/

				uwb_location::RangeArray rangedata;
				rangedata.data[0] = range[0];
				rangedata.data[1] = range[1];
				rangedata.data[2] = range[2];
				rangedata.data[3] = range[3];
				pub.publish(rangedata);

				bzero(receive_buf, sizeof(receive_buf));

				DataRecord = 0;
			}
		}
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "uwb_range_node"); // 发布uwb节点
	// 创建句柄
	ros::NodeHandle nh("~");

	std::string port_name;
	nh.param("port_name", port_name, std::string("/dev/ttyUSB0"));

	ros::Publisher range_pub_ =
	    nh.advertise<uwb_location::RangeArray>("/uwb/range", 100);

	// 创建一个serial类
	serial::Serial sp;
	// 创建timeout
	serial::Timeout to = serial::Timeout::simpleTimeout(11);
	// 设置要打开的串口名称
	sp.setPort(port_name);
	// 设置串口通信的波特率
	sp.setBaudrate(115200);
	// 串口设置timeout
	sp.setTimeout(to);

	try {
		// 打开串口
		sp.open();
	} catch (serial::IOException& e) {
		ROS_ERROR_STREAM("Unable to open port.");
		return -1;
	}

	// 判断串口是否打开成功
	if (sp.isOpen()) {
		ROS_INFO_STREAM("port is opened.");
	} else {
		return -1;
	}

	while (ros::ok()) {
		ros::spinOnce();
		// 获取缓冲区内的字节数
		size_t len = sp.available();

		if (len > 0) {
			unsigned char usart_buf[1024] = {0};
			sp.read(usart_buf, len);

			unsigned char* pbuf;
			unsigned char buf[2014] = {0};

			pbuf = (unsigned char*)usart_buf;
			memcpy(&buf[0], pbuf, len);

			int reallength = len;
			int i;
			if (reallength != 0) {
				for (i = 0; i < reallength; i++) {
					BufDataFromCtrl[BufCtrlPosit_w] = buf[i];

					BufCtrlPosit_w = (BufCtrlPosit_w == (MAX_DATA_NUM - 1))
					                     ? 0
					                     : (1 + BufCtrlPosit_w);
				}
			}
			CtrlSerDataDeal(sp, range_pub_);
		}
	}
	// 关闭串口
	sp.close();
	return 0;
}
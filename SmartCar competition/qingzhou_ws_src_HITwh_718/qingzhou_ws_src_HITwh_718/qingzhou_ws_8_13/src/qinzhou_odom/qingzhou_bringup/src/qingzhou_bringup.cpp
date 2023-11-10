#include "qingzhou_bringup/qingzhou_bringup.h"

long long LeftticksPerMeter = 0;       //左轮编码器每米脉冲数
long long rightticksPerMeter = 0;      //右轮编码器每米脉冲数
long long LeftticksPer2PI = 0;         //左轮每圈编码器脉冲数
long long rightticksPer2PI = 0;        //右轮每圈编码器脉冲数

/*新写变量*/
long long old_LeftticksPerMeter = 0;   //上一轮循环左轮每圈编码器脉冲数
long long old_rightticksPerMeter = 0;  //上一轮循环右轮每圈编码器脉冲数

// 构造函数，初始化
actuator::actuator(ros::NodeHandle handle) 
{
	m_baudrate = 115200;               //波特率
	m_serialport = "/dev/stm32board";  //串口名
	linearSpeed = 0;                   //线速度
	angularSpeed = 0;                  //角速度
	batteryVoltage = 0;                //电池电压
	ticksPerMeter = 0;                 //一米脉冲数
	ticksPer2PI = 0;                   //每圈脉冲数
	encoderLeft = 0;                   //左编码器
	encoderRight = 0;                  //右编码器
	velDeltaTime = 0;
	calibrate_lineSpeed = 0;           //标定标志位
	calibrate_angularSpeed = 0;
    //初始化moveBaseControl的数值为0
	memset(&moveBaseControl, 0, sizeof(sMartcarControl));

	handle.param("mcubaudrate", m_baudrate, m_baudrate);
	handle.param("mcuserialport", m_serialport, std::string("/dev/ttyUSB0"));
	handle.param("calibrate_lineSpeed", calibrate_lineSpeed, calibrate_lineSpeed);
	handle.param("calibrate_angularSpeed", calibrate_angularSpeed, calibrate_angularSpeed);
	handle.param("ticksPerMeter", ticksPerMeter, ticksPerMeter);
	handle.param("ticksPer2PI", ticksPer2PI, ticksPer2PI);

	try {//初始化一个串口

		std::cout << "[qingzhou_actuator-->]" << "Serial initialize start!" << std::endl;
		ser.setPort(m_serialport.c_str());
		ser.setBaudrate(m_baudrate);
                
		serial::Timeout to = serial::Timeout::simpleTimeout(30);
		ser.setTimeout(to);
		ser.open();

        }	

	catch (serial::IOException& e) {
		std::cout << "[qingzhou_actuator-->]" << "Unable to open port!" << std::endl;
	}
	if (ser.isOpen()) {
		std::cout << "[qingzhou_actuator-->]" << "Serial initialize successfully!" << std::endl;
	}
	else {
		std::cout << "[qingzhou_actuator-->]" << "Serial port failed!" << std::endl;
	}

	sub_move_base = handle.subscribe("cmd_vel", 3, &actuator::callback_move_base, this);//订阅速度话题  ###1
	//发布话题
	pub_imu = handle.advertise<sensor_msgs::Imu>("raw", 5);
	pub_mag = handle.advertise<sensor_msgs::MagneticField>("imu/mag", 5);
	pub_odom = handle.advertise<nav_msgs::Odometry>("odom", 5);
	pub_battery = handle.advertise<std_msgs::Float32>("battery", 10);
}

//析构函数
actuator::~actuator() {

}
/*
函数：订阅的回调函数
传入：cmd_vel数据(Twist格式类型)
数据转化：x(m/s)*74->线速度（cm/s）(这里尚有疑问)
         根据x是线速度（m/s），z是角速度（rad/s）转换出中心转弯角度
输出:线速度，角速度，传输到下位机的线速度以及角度
*/
void actuator::callback_move_base(const geometry_msgs::Twist::ConstPtr& msg)    //对应cmd_vel话题，对应geometry_msgs/Twist消息
{
	memset(&moveBaseControl, 0, sizeof(sMartcarControl));                       //清零movebase数据存储区

	float v = (float)msg->linear.x;                                                    //move_base算得的线速度
	float w = (float)msg->angular.z;                                                   //move_base算得的角速度
    /*半径R*/
    float R;R=v/w;
	cout<<"速度v: "<<v<<"角速度w: "<<w<<"理想半径: "<<R;
	moveBaseControl.TargetSpeed = v * 32 / 0.43;                                //计算目标线速度 1对应74cm/s
	R=v/w;
	float R_single=1;
	if(R<0){
		R_single=-1;
		R=-R;
	}
    /*下新代码*/
//	moveBaseControl.TargetSpeed = v * 100;                                //计算目标线速度 1对应74cm/s
	/*半径对应转换,半径为1时v/w约为4/3，半径为2时v/w约为2.1*/
	// moveBaseControl.TargetAngle = round(atan(w * CARL / v) * 57.3);             //计算目标角度
	/*新代码映射换算*/
	if(R>0.72&&R<=3.08){
		if(R<0.92){
			R=1+(4.0/3.0-1)*(R-0.72)/0.2;
		}
		else if(R<1.27){
			R=4.0/3.0+(2-4.0/3.0)*(R-0.92)/0.35;
		}
		else if(R<1.47){
			R=2+(20.0/9.0-2)*(R-1.27)/0.2;
		}
		else if(R<1.84){
			R=20.0/9.0+(8.0/3.0-20.0/9.0)*(R-1.47)/0.37;
		}
		else if(R<2.16){
			R=8.0/3.0+(10.0/3.0-8.0/3.0)*(R-1.84)/0.32;
		}
		else if(R<2.36){
			R=10.0/3.0+(40.0/11-10.0/3.0)*(R-2.16)/0.2;
		}
		else if(R<2.65){
			R=40.0/11+(4.0-40.0/11)*(R-2.36)/0.29;
		}
		else {
			R=4.0+(40.0/9.0-4.0)*(R-2.65)/0.43;
		}
	}
	R=R*R_single;
	//取消四舍五入
	// moveBaseControl.TargetAngle = atan(w * CARL / v) * 57.3;             //计算目标角度
	moveBaseControl.TargetAngle = atan(CARL / R) * 57.3;             //计算目标角度
	if(R<-3.08||R>3.08){
		moveBaseControl.TargetAngle*=0.5;
	}
	cout<<"角度： "<<moveBaseControl.TargetAngle<<endl;
	moveBaseControl.TargetAngle += 60;                                          //stm32 program has subtract 60
    /*输出运算后的角度以及线速度的绝对值*/
//	printf("%.2f,%.2f,%d,%d\n", msg->linear.x, msg->angular.z,
//		abs(moveBaseControl.TargetSpeed), abs(moveBaseControl.TargetAngle));

}

void actuator::run() {
	int run_rate = 50;
	ros::Rate rate(run_rate);

	double x = 0.0;                 //x坐标                       
	double y = 0.0;                 //y坐标
	double th = 0.0;

	ros::Time current_time, last_time;

	while (ros::ok()) {
		ros::spinOnce();
		current_time = ros::Time::now();                  //获得当前时间
		velDeltaTime = (current_time - last_time).toSec();//转换成秒
		last_time = ros::Time::now();

		recvCarInfoKernel();                              //接收stm32发来的数据
		pub_9250();                                       //发布imu数据

		currentBattery.data = batteryVoltage;
		pub_battery.publish(currentBattery);

#if 1
		if (encoderLeft > 220 || encoderLeft < -220) encoderLeft = 0;
		if (encoderRight > 220 || encoderRight < -220) encoderRight = 0;
		/*
		这里的正负数的调整是和底层正负数据相对应的
		就是为了防止编码器的接线接反（其实轻舟就是接反了。。。。。。）
		*/
		//encoderLeft = -encoderLeft;
		//encoderRight = -encoderRight;
        /*旧*/
		detEncode = (encoderLeft + encoderRight) / 2;
		detdistance = detEncode / ticksPerMeter;
		detth = (encoderRight - encoderLeft) * 2 * PI / ticksPer2PI; //计算当前角度,通过标定获得ticksPer2PI
		/*新*/
		// detdistance=(LeftticksPerMeter+rightticksPerMeter-old_LeftticksPerMeter-old_rightticksPerMeter)/2/ticksPerMeter;
		// detth=(-LeftticksPerMeter+rightticksPerMeter+old_LeftticksPerMeter-old_rightticksPerMeter)* 2 * PI / ticksPer2PI;
        /*旧*/
		// linearSpeed = detdistance / velDeltaTime;
		// angularSpeed = detth / velDeltaTime;
        /*新*/
         linearSpeed = (LeftticksPerMeter+rightticksPerMeter-old_LeftticksPerMeter-old_rightticksPerMeter)/2/ticksPerMeter/velDeltaTime;
		 angularSpeed = (-LeftticksPerMeter+rightticksPerMeter+old_LeftticksPerMeter-old_rightticksPerMeter)* 2 * PI / ticksPer2PI / velDeltaTime;
         old_LeftticksPerMeter=LeftticksPerMeter;
		 old_rightticksPerMeter=rightticksPerMeter;
		if (detdistance != 0) {
			x += detdistance * cos(th);                        //x坐标
			y += detdistance * sin(th);                        //y坐标
		}
		if (detth != 0) {
			th += detth;                                       //是和xy有关的偏角（不是位姿）
		}

		if (calibrate_lineSpeed == 1) {
			printf("x=%.2f,y=%.2f,th=%.2f,linearSpeed=%.2f,,detEncode=%.2f,LeftticksPerMeter = %lld,rightticksPerMeter = %lld,batteryVoltage = %.2f,encoderLeft= %d,encoderRight= %d\n", x, y, th, linearSpeed, detEncode, LeftticksPerMeter, rightticksPerMeter, batteryVoltage,encoderLeft,encoderRight);
		}

		//send command to stm32
		sendCarInfoKernel();
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		nav_msgs::Odometry odom;                               //创建nav_msgs::Odometry类型的消息odom
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		odom.twist.twist.linear.x = linearSpeed;               //线速度
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.linear.z = 0;
		odom.twist.twist.angular.x = 0;
		odom.twist.twist.angular.y = 0;
		odom.twist.twist.angular.z = angularSpeed;             //角速度
		if (encoderLeft == 0 && encoderRight == 0) {
			odom.pose.covariance = { 1e-9, 0, 0, 0, 0, 0,
									0, 1e-3, 1e-9, 0, 0, 0,
									0, 0, 1e6, 0, 0, 0,
									0, 0, 0, 1e6, 0, 0,
									0, 0, 0, 0, 1e6, 0,
									0, 0, 0, 0, 0, 1e-9 };

			odom.twist.covariance = { 1e-9, 0, 0, 0, 0, 0,
									0, 1e-3, 1e-9, 0, 0, 0,
									0, 0, 1e6, 0, 0, 0,
									0, 0, 0, 1e6, 0, 0,
									0, 0, 0, 0, 1e6, 0,
									0, 0, 0, 0, 0, 1e-9 };
		}
		else {
			odom.pose.covariance = { 1e-3, 0, 0, 0, 0, 0,
									0, 1e-3, 0, 0, 0, 0,
									0, 0, 1e6, 0, 0, 0,
									0, 0, 0, 1e6, 0, 0,
									0, 0, 0, 0, 1e6, 0,
									0, 0, 0, 0, 0, 1e3 };

			odom.twist.covariance = { 1e-3, 0, 0, 0, 0, 0,
									0, 1e-3, 0, 0, 0, 0,
									0, 0, 1e6, 0, 0, 0,
									0, 0, 0, 1e6, 0, 0,
									0, 0, 0, 0, 1e6, 0,
									0, 0, 0, 0, 0, 1e3 };
		}
		pub_odom.publish(odom);

#endif
		rate.sleep();
	}
}

//发送小车数据到下位机
void actuator::sendCarInfoKernel() {
	unsigned char buf[23] = { 0 };
	buf[0] = 0xa5;
	buf[1] = 0x5a;
	buf[2] = 0x06;

	float TargetAngle, TargetSpeed;
	TargetAngle =-(moveBaseControl.TargetAngle - 60);
	/*新*/
    float targetangle=TargetAngle/2;
	char* dataptr = (char*)&TargetAngle;
//	char* dataptr = (char*)&targetangle;
	int datasize = sizeof(TargetAngle);
	int sizei = 3;
	while (datasize--) {
		buf[sizei] = *dataptr++;
		sizei++;
	}
	TargetSpeed = moveBaseControl.TargetSpeed;
    /*速度映射*/
    float target_speed=TargetSpeed;
	if(target_speed>-3.72&&target_speed<3.72)
		target_speed=0;
	else if(target_speed<0)
	{
        if(target_speed>-14.88&&target_speed<-7.44){
        target_speed-=7.44;
	    /*进入判断*/
//		ROS_INFO("-1dasasdas");
        }
		else if(target_speed>-7.44)
			target_speed-=11.16;
	}
	else
	{
        if(target_speed<7.44)
			target_speed += 11.16;
		else if(target_speed<14.88&&target_speed>7.44){
        	target_speed+=7.44;
			/*进入判断*/
//			ROS_INFO("1dkajbdsiabd");
        }
	}
	char* datatsp = (char*)&target_speed;
    /*非速度映射*/
//	char* datatsp = (char*)&TargetSpeed;
	int datasize_speed = sizeof(TargetSpeed);
	while (datasize_speed--) {
		buf[sizei] = *datatsp++;
		sizei++;
	}
	//buf[4] = (int)abs(moveBaseControl.TargetAngle);	    //targetangle
	//buf[5] = (int)abs(moveBaseControl.TargetSpeed);	    //targetSpeed
	buf[sizei] = 0xc4;
	sizei++;
	buf[sizei] = 0x03;
	sizei++;
	buf[sizei] = 0x0d;
	/*unsigned char sum = 0;
	for(int i = 2; i < 19; ++i)
		sum += buf[i];
	buf[9] = (unsigned char)(sum); */
	size_t writesize = ser.write(buf, sizei + 1);
	// ROS_INFO("target_v:%f target_angle:%f\n",target_speed,TargetAngle);
}

//接收下位机发送来的数据
void actuator::recvCarInfoKernel() {
	std::string recvstr;
	unsigned char tempdata, lenrecv;
	unsigned char count, last_data, last_last_data, last_last_last_data;
	unsigned char str[100];
	bool recvflag = false;
	bool recvd_flag = false;
	memset(&str, 0, sizeof(str));
	ros::Time begin_time = ros::Time::now();
	double clustering_time = 0;

	while (1) {
		clustering_time = (ros::Time::now() - begin_time).toSec(); //计算时间差，转换成秒
		if (clustering_time > 1) {
			recvd_flag = false;
			break;
		}

		recvstr = ser.read(1);
		if ((int)recvstr.size() != 1)
			continue;

		tempdata = recvstr[0];
		if (last_last_last_data == 0xa5 && last_last_data == 0x5a) {
			lenrecv = last_data;
			recvflag = true;
			count = 0;
		}
		if (recvflag) {
			str[count] = tempdata;
			count++;
			if (count == 48) {
				recvflag = false;
				recvd_flag = true;
				break;
			}
		}
		last_last_last_data = last_last_data;
		last_last_data = last_data;
		last_data = tempdata;
	}

	if (recvd_flag) {                                                  //数据解析，接收到的数据转存
		memcpy(&encoderLeft, str, 4);
		serial::Timeout to = serial::Timeout::simpleTimeout(30);
		/*serial::Timeout*/ to = serial::Timeout::simpleTimeout(30);
		/*serial::Timeout*/ to = serial::Timeout::simpleTimeout(30);
		memcpy(&encoderRight, str + 4, 4);
		memcpy(&batteryVoltage, str + 8, 4);

		/*serial::Timeout*/ to = serial::Timeout::simpleTimeout(30);
		memcpy(&tempaccelX, str + 12, 4);
		memcpy(&tempaccelY, str + 16, 4);
		memcpy(&tempaccelZ, str + 20, 4);

		memcpy(&tempgyroX, str + 24, 4);
		memcpy(&tempgyroY, str + 28, 4);
		memcpy(&tempgyroZ, str + 32, 4);

		memcpy(&tempmagX, str + 36, 4);
		memcpy(&tempmagY, str + 40, 4);
		memcpy(&tempmagZ, str + 44, 4);

		accelX = (float)tempaccelX / 2048 * 9.8;                         //线加速度处理	
		accelY = (float)tempaccelY / 2048 * 9.8;
		accelZ = (float)tempaccelZ / 2048 * 9.8;


		gyroX = (float)tempgyroX / 16.4 / 57.3;                          //角速度处理
		gyroY = (float)tempgyroY / 16.4 / 57.3;
		gyroZ = (float)tempgyroZ / 16.4 / 57.3;

		magX = (float)tempmagX * 0.14;                                 //磁力计处理
		magY = (float)tempmagY * 0.14;
		magZ = (float)tempmagZ * 0.14;

		if (encoderLeft > 220 || encoderLeft < -220) encoderLeft = 0; //判断编码器脉冲数是否在正确范围
		if (encoderRight > 220 || encoderRight < -220) encoderRight = 0;
                if (encoderRight-encoderLeft>-4&&encoderRight-encoderLeft<0){
                encoderLeft=encoderRight;

                }
		LeftticksPerMeter += encoderLeft;                            //获得左轮总脉冲数
		rightticksPerMeter += encoderRight;                          //获得右轮总脉冲数
	}
}

//发布imu函数
void actuator::pub_9250() {
	sensor_msgs::Imu imuMsg;
	sensor_msgs::MagneticField magMsg;

	ros::Time current_time = ros::Time::now();

	imuMsg.header.stamp = current_time;
	imuMsg.header.frame_id = "imu_link";
	imuMsg.angular_velocity.x = gyroX;
	imuMsg.angular_velocity.y = gyroY;
	imuMsg.angular_velocity.z = gyroZ;
	imuMsg.angular_velocity_covariance = {
	  0.04,0.0,0.0,
	  0.0,0.04,0.0,
	  0.0,0.0,0.04
	};

	imuMsg.linear_acceleration.x = accelX;
	imuMsg.linear_acceleration.y = accelY;
	imuMsg.linear_acceleration.z = accelZ;
	imuMsg.linear_acceleration_covariance = {
	  0.04,0.0,0.0,
	  0.0,0.04,0.0,
	  0.0,0.0,0.04
	};
	pub_imu.publish(imuMsg);                       //发布imuMsg 

	magMsg.header.stamp = current_time;
	magMsg.header.frame_id = "base_link";
	magMsg.magnetic_field.x = magX;
	magMsg.magnetic_field.y = magY;
	magMsg.magnetic_field.z = magZ;
	magMsg.magnetic_field_covariance = {
	  0.0,0.0,0.0,
	  0.0,0.0,0.0,
	  0.0,0.0,0.0
	};
	pub_mag.publish(magMsg);                       //发布magMsg
}

#include "ros/ros.h"
#include <string.h>
#include <ros/time.h>
#include <serial/serial.h>
#include "ArduinoJson.h"
#include <geometry_msgs/TwistWithCovarianceStamped.h>// encoder twist msg
#include <sensor_msgs/Imu.h>// imu msg
#include <sensor_msgs/MagneticField.h>// imu msg
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
// #include <dynamic_reconfigure/server.h>
// #include <mrc10100/mrc10100Config.h>

#include <boost/thread/thread.hpp>

class Mrc10100
{
	public:
		Mrc10100();
		// void reconfigureCallback(mrc10100::mrc10100Config &config, uint32_t level);
		void cmdCallback(const geometry_msgs::Twist::ConstPtr& twist, std::string topic);
		void gpoWriteCallback(const std_msgs::Int16MultiArray::ConstPtr& msg, std::string topic);
		void run();
		uint16_t modbusCRC(uint8_t arrayByte[], uint8_t sizeByte, bool order);

	private:
		ros::NodeHandle node_;
		ros::Publisher encoder_publisher;
		ros::Publisher imu_publisher;
		ros::Publisher magnetic_field_publisher;
		ros::Publisher gpi_publisher;
		ros::Publisher gpo_read_publisher;
		ros::Publisher analog_publisher;
		ros::Publisher voltage_current_publisher;
		ros::Publisher count_publisher;
		ros::Publisher status_publisher;
		ros::Publisher debug_publisher;
		ros::Subscriber cmd_subscriber;
		ros::Subscriber gpo_write_subscriber;

		std::string encoder_topic;
		std::string imu_topic;
		std::string magnetic_field_topic;
		std::string gpi_topic;
		std::string gpo_read_topic;
		std::string analog_topic;
		std::string voltage_current_topic;
		std::string count_topic;
		std::string status_topic;
		std::string debug_topic;
		std::string cmd_topic;
		std::string gpo_write_topic;

		std::string port;
		int baud_rate;
		std::string base_frame_id;

		double pid_x_p;
		double pid_x_i;
		double pid_x_d;
		double pid_x_n;
		double pid_x_high;
		double pid_x_low;
		bool pid_x_windup;

		double pid_z_p;
		double pid_z_i;
		double pid_z_d;
		double pid_z_n;
		double pid_z_high;
		double pid_z_low;
		bool pid_z_windup;

		double wheel_distance;
		double wheel_right_size;
		double wheel_left_size;

		double encoder_wheel_right_cpr;
		double encoder_wheel_right_n;
		double encoder_wheel_left_cpr ;
		double encoder_wheel_left_n;

		double imu_acc_bias_x;
		double imu_acc_bias_y;
		double imu_acc_bias_z;
		double imu_acc_scale_x;
		double imu_acc_scale_y;
		double imu_acc_scale_z;

		double imu_gyro_bias_x;
		double imu_gyro_bias_y;
		double imu_gyro_bias_z;

		double imu_mag_bias_x;
		double imu_mag_bias_y;
		double imu_mag_bias_z;
		double imu_mag_scale_x;
		double imu_mag_scale_y;
		double imu_mag_scale_z;

		bool advance;
		bool debug;

		int16_t gpo_write_temp[6] = {0, 0, 0, 0, 0, 0};
		bool gpo_write_update = false;
		double cmd_temp[2] = {0, 0};
		int16_t status_temp[1] = {0};

		serial::Serial* ser;
		bool serial_fail_retry = false;
		ros::Time serial_retry_timeout = ros::Time::now();
		ros::Time receive_timeout = ros::Time::now();
		std::string rev = "";
		bool one_time_send = false;
		bool receive_status = false;
		bool success_receive  = false;

		boost::thread* thr0;
};

Mrc10100::Mrc10100()
{
	ros::NodeHandle nh("~");

	//Setting class parameters
	if(!nh.getParam("encoder_topic", encoder_topic))
		encoder_topic = "/mrc10100/encoder";
	if(!nh.getParam("imu_topic", imu_topic))
		imu_topic = "/mrc10100/imu";
	if(!nh.getParam("magnetic_field_topic", magnetic_field_topic))
		magnetic_field_topic = "/mrc10100/magnetic_field";
	if(!nh.getParam("gpi_topic", gpi_topic))
		gpi_topic = "/mrc10100/gpi";
	if(!nh.getParam("gpo_read_topic", gpo_read_topic))
		gpo_read_topic = "/mrc10100/gpo_read";
	if(!nh.getParam("analog_topic", analog_topic))
		analog_topic = "/mrc10100/analog";
	if(!nh.getParam("voltage_current_topic", voltage_current_topic))
		voltage_current_topic = "/mrc10100/voltage_current";
	if(!nh.getParam("count_topic", count_topic))
		count_topic = "/mrc10100/count";
	if(!nh.getParam("status_topic", status_topic))
		status_topic = "/mrc10100/status";
	if(!nh.getParam("debug_topic", debug_topic))
		debug_topic = "/mrc10100/debug";

	if(!nh.getParam("cmd_topic", cmd_topic))
		cmd_topic = "/mrc10100/cmd_vel";
	if(!nh.getParam("gpo_write_topic", gpo_write_topic))
		gpo_write_topic = "/mrc10100/gpo_write";

	if(!nh.getParam("port", port))
		port = "/dev/ttyACM0";
	if(!nh.getParam("baud_rate", baud_rate))
		baud_rate = 115200;
	if(!nh.getParam("base_frame_id", base_frame_id))
		base_frame_id = "/base_link";

	if(!nh.getParam("pid_x_p", pid_x_p))
		pid_x_p = 217;
	if(!nh.getParam("pid_x_i", pid_x_i))
		pid_x_i = 2407;
	if(!nh.getParam("pid_x_d", pid_x_d))
		pid_x_d = 13;
	if(!nh.getParam("pid_x_n", pid_x_n))
		pid_x_n = 13;
	if(!nh.getParam("pid_x_high", pid_x_high))
		pid_x_high = 1000;
	if(!nh.getParam("pid_x_low", pid_x_low))
		pid_x_low = -1000;
	if(!nh.getParam("pid_x_windup", pid_x_windup))
		pid_x_windup = true;

	if(!nh.getParam("pid_z_p", pid_z_p))
		pid_z_p = 217;
	if(!nh.getParam("pid_z_i", pid_z_i))
		pid_z_i = 2407;
	if(!nh.getParam("pid_z_d", pid_z_d))
		pid_z_d = 13;
	if(!nh.getParam("pid_z_n", pid_z_n))
		pid_z_n = 13;
	if(!nh.getParam("pid_z_high", pid_z_high))
		pid_z_high = 1000;
	if(!nh.getParam("pid_z_low", pid_z_low))
		pid_z_low = -1000;
	if(!nh.getParam("pid_z_windup", pid_z_windup))
		pid_z_windup = true;

	if(!nh.getParam("wheel_distance", wheel_distance))
		wheel_distance = 0.0745;
	if(!nh.getParam("wheel_right_size", wheel_right_size))
		wheel_right_size = 0.0745;
	if(!nh.getParam("wheel_left_size", wheel_left_size))
		wheel_left_size = 0.0745;

	if(!nh.getParam("encoder_wheel_right_cpr", encoder_wheel_right_cpr))
		encoder_wheel_right_cpr = 536;
	if(!nh.getParam("encoder_wheel_right_n", encoder_wheel_right_n))
		encoder_wheel_right_n = 13;
	if(!nh.getParam("encoder_wheel_left_cpr", encoder_wheel_left_cpr))
		encoder_wheel_left_cpr = 536;
	if(!nh.getParam("encoder_wheel_left_n", encoder_wheel_left_n))
		encoder_wheel_left_n = 13;

	if(!nh.getParam("imu_acc_bias_x", imu_acc_bias_x))
		imu_acc_bias_x = 13;
	if(!nh.getParam("imu_acc_bias_y", imu_acc_bias_y))
		imu_acc_bias_y = 13;
	if(!nh.getParam("imu_acc_bias_z", imu_acc_bias_z))
		imu_acc_bias_z = 13;
	if(!nh.getParam("imu_acc_scale_x", imu_acc_scale_x))
		imu_acc_scale_x = 13;
	if(!nh.getParam("imu_acc_scale_y", imu_acc_scale_y))
		imu_acc_scale_y = 13;
	if(!nh.getParam("imu_acc_scale_z", imu_acc_scale_z))
		imu_acc_scale_z = 13;

	if(!nh.getParam("imu_gyro_bias_x", imu_gyro_bias_x))
		imu_gyro_bias_x = 13;
	if(!nh.getParam("imu_gyro_bias_y", imu_gyro_bias_y))
		imu_gyro_bias_y = 13;
	if(!nh.getParam("imu_gyro_bias_z", imu_gyro_bias_z))
		imu_gyro_bias_z = 13;

	if(!nh.getParam("imu_mag_bias_x", imu_mag_bias_x))
		imu_mag_bias_x = 13;
	if(!nh.getParam("imu_mag_bias_y", imu_mag_bias_y))
		imu_mag_bias_y = 13;
	if(!nh.getParam("imu_mag_bias_z", imu_mag_bias_z))
		imu_mag_bias_z = 13;
	if(!nh.getParam("imu_mag_scale_x", imu_mag_scale_x))
		imu_mag_scale_x = 13;
	if(!nh.getParam("imu_mag_scale_y", imu_mag_scale_y))
		imu_mag_scale_y = 13;
	if(!nh.getParam("imu_mag_scale_z", imu_mag_scale_z))
		imu_mag_scale_z = 13;

	if(!nh.getParam("advance", advance))
		advance = false;
	if(!nh.getParam("debug", debug))
		debug = false;

	encoder_publisher = node_.advertise<geometry_msgs::TwistWithCovarianceStamped> (encoder_topic.c_str(), 1, false);
	imu_publisher = node_.advertise<sensor_msgs::Imu> (imu_topic.c_str(), 1, false);
	magnetic_field_publisher = node_.advertise<sensor_msgs::MagneticField> (magnetic_field_topic.c_str(), 1, false);
	gpi_publisher = node_.advertise<std_msgs::Int16MultiArray> (gpi_topic.c_str(), 1, false);
	gpo_read_publisher = node_.advertise<std_msgs::Int16MultiArray> (gpo_read_topic.c_str(), 1, false);
	analog_publisher = node_.advertise<std_msgs::Float64MultiArray> (analog_topic.c_str(), 1, false);
	voltage_current_publisher = node_.advertise<std_msgs::Float64MultiArray> (voltage_current_topic.c_str(), 1, false);
	count_publisher = node_.advertise<std_msgs::Int32MultiArray> (count_topic.c_str(), 1, false);
	status_publisher = node_.advertise<std_msgs::Int16MultiArray> (status_topic.c_str(), 1, false);
	if(debug)
	{
		debug_publisher = node_.advertise<std_msgs::String> (debug_topic.c_str(), 1, false);
	}

	cmd_subscriber = node_.subscribe<geometry_msgs::Twist> (cmd_topic.c_str(), 1, boost::bind(&Mrc10100::cmdCallback,this, _1, cmd_topic));
	gpo_write_subscriber = node_.subscribe<std_msgs::Int16MultiArray> (gpo_write_topic.c_str(), 1, boost::bind(&Mrc10100::gpoWriteCallback,this, _1, gpo_write_topic));

	ser = new serial::Serial();
	// thr0 = new boost::thread(boost::bind(&Mrc10100::run, this));
}

// void Mrc10100::reconfigureCallback(mrc10100::mrc10100Config &config, uint32_t level)
// {
// 	this->pid_x_p = config.pid_x_p;
// 	this->pid_x_i = config.pid_x_i;
// 	this->pid_x_d = config.pid_x_d;
// 	this->pid_x_n = config.pid_x_n;
// 	this->pid_x_high = config.pid_x_high;
// 	this->pid_x_low = config.pid_x_low;
// 	this->pid_x_windup = config.pid_x_windup;

// 	this->pid_z_p = config.pid_z_p;
// 	this->pid_z_i = config.pid_z_i;
// 	this->pid_z_d = config.pid_z_d;
// 	this->pid_z_n = config.pid_z_n;
// 	this->pid_z_high = config.pid_z_high;
// 	this->pid_z_low = config.pid_z_low;
// 	this->pid_z_windup = config.pid_z_windup;

// 	this->wheel_distance = config.wheel_distance;
// 	this->wheel_right_size = config.wheel_right_size;
// 	this->wheel_left_size = config.wheel_left_size;

// 	this->encoder_wheel_right_cpr = config.encoder_wheel_right_cpr;
// 	this->encoder_wheel_right_n = config.encoder_wheel_right_n;
// 	this->encoder_wheel_left_cpr = config.encoder_wheel_left_cpr;
// 	this->encoder_wheel_left_n = config.encoder_wheel_left_n;

// 	this->imu_acc_bias_x = config.imu_acc_bias_x;
// 	this->imu_acc_bias_y = config.imu_acc_bias_y;
// 	this->imu_acc_bias_z = config.imu_acc_bias_z;
// 	this->imu_acc_scale_x = config.imu_acc_scale_x;
// 	this->imu_acc_scale_y = config.imu_acc_scale_y;
// 	this->imu_acc_scale_z = config.imu_acc_scale_z;

// 	this->imu_gyro_bias_x = config.imu_gyro_bias_x;
// 	this->imu_gyro_bias_y = config.imu_gyro_bias_y;
// 	this->imu_gyro_bias_z = config.imu_gyro_bias_z;

// 	this->imu_mag_bias_x = config.imu_mag_bias_x;
// 	this->imu_mag_bias_y = config.imu_mag_bias_y;
// 	this->imu_mag_bias_z = config.imu_mag_bias_z;
// 	this->imu_mag_scale_x = config.imu_mag_scale_x;
// 	this->imu_mag_scale_y = config.imu_mag_scale_y;
// 	this->imu_mag_scale_z = config.imu_mag_scale_z;

// 	this->advance = config.advance;
// }

void Mrc10100::cmdCallback(const geometry_msgs::Twist::ConstPtr& twist, std::string topic)
{
	cmd_temp[0] = twist->linear.x;
	cmd_temp[1] = twist->angular.z;
}

void Mrc10100::gpoWriteCallback(const std_msgs::Int16MultiArray::ConstPtr& msg, std::string topic)
{
	if(msg->layout.dim[0].size == 6)
	{
		std_msgs::Int16MultiArray gpo_read_msg;
		gpo_read_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
		gpo_read_msg.layout.dim[0].size = msg->layout.dim[0].size;
		gpo_read_msg.layout.dim[0].stride = 1;
		gpo_read_msg.layout.dim[0].label = "gpo";
		for(int i=0; i<msg->layout.dim[0].size; i++)
		{
			gpo_read_msg.data.push_back( msg->data[i] );
			gpo_write_temp[i] = msg->data[i];
		}
		gpo_read_publisher.publish(gpo_read_msg);
		gpo_write_update = true;
	}
}

void Mrc10100::run()
{
	ros::Rate loop_rate(50);

	while (ros::ok())
	{
		ros::spinOnce();
		if(!ser->isOpen())
		{
			if((ros::Time::now() - serial_retry_timeout).toSec() >= 1)
			{
				one_time_send = true;
				receive_status  = true;
				success_receive  = false;
				try
				{
					ser->setPort(port);
					ser->setBaudrate(baud_rate);
					serial::Timeout to = serial::Timeout::simpleTimeout(1000);
					ser->setTimeout(to);
					ser->open();
					ROS_INFO_STREAM("mrc10100: success to open port "<<port);
					while(ros::ok() && ser->available())
					{
						ser->readline(ser->available(),"");
					}
				}
				catch(serial::PortNotOpenedException& e)
				{
					if(!serial_fail_retry)
					{
						ROS_ERROR_STREAM("mrc10100: fail to open port "<<port<<", will retry every seconds");
						// ROS_ERROR_STREAM(e.what());
					}
				}
				catch(serial::SerialException& e)
				{
					if(!serial_fail_retry)
					{
						ROS_ERROR_STREAM("mrc10100: fail to read, will retry every seconds");
						// ROS_ERROR_STREAM(e.what());
					}
				}
				catch(serial::IOException& e)
				{
					if(!serial_fail_retry)
					{
						ROS_ERROR_STREAM("mrc10100: fail to find serial port "<<port<<", will retry every seconds");
						// ROS_ERROR_STREAM(e.what());
					}
				}
				catch(...)
				{
					if(!serial_fail_retry)
					{
						ROS_ERROR_STREAM("mrc10100: unknown error to open serial port, will retry every seconds");
					}
				}
				if(!ser->isOpen())
				{
					serial_fail_retry = true;
					serial_retry_timeout = ros::Time::now();
				}
			}
		}
		else
		{
			serial_fail_retry = false;
			try
			{
				if(receive_status)
				{
					StaticJsonDocument<6000> root;
					std::string data = "";
					root["S"] = 1;
					if(one_time_send)
					{
						JsonArray pid_x = root.createNestedArray("PIDX#");
						pid_x.add(std::to_string(pid_x_p));
						pid_x.add(std::to_string(pid_x_i));
						pid_x.add(std::to_string(pid_x_d));
						pid_x.add(std::to_string(pid_x_n));
						pid_x.add(std::to_string(pid_x_high));
						pid_x.add(std::to_string(pid_x_low));
						pid_x.add((uint8_t)pid_x_windup);
						JsonArray pid_z = root.createNestedArray("PIDZ#");
						pid_z.add(std::to_string(pid_z_p));
						pid_z.add(std::to_string(pid_z_i));
						pid_z.add(std::to_string(pid_z_d));
						pid_z.add(std::to_string(pid_z_n));
						pid_z.add(std::to_string(pid_z_high));
						pid_z.add(std::to_string(pid_z_low));
						pid_z.add((uint8_t)pid_z_windup);
						JsonArray base = root.createNestedArray("BASE#");
						base.add(std::to_string(wheel_distance));
						base.add(std::to_string(wheel_right_size));
						base.add(std::to_string(wheel_left_size));
						JsonArray enc = root.createNestedArray("ENC#");
						enc.add(std::to_string(encoder_wheel_right_cpr));
						enc.add(std::to_string(encoder_wheel_right_n));
						enc.add(std::to_string(encoder_wheel_left_cpr));
						enc.add(std::to_string(encoder_wheel_left_n));
						JsonArray acc = root.createNestedArray("AB#");
						acc.add(std::to_string(imu_acc_bias_x));
						acc.add(std::to_string(imu_acc_bias_y));
						acc.add(std::to_string(imu_acc_bias_z));
						acc.add(std::to_string(imu_acc_scale_x));
						acc.add(std::to_string(imu_acc_scale_y));
						acc.add(std::to_string(imu_acc_scale_z));
						JsonArray gyro = root.createNestedArray("GB#");
						gyro.add(std::to_string(imu_gyro_bias_x));
						gyro.add(std::to_string(imu_gyro_bias_y));
						gyro.add(std::to_string(imu_gyro_bias_z));
						JsonArray mag = root.createNestedArray("MB#");
						mag.add(std::to_string(imu_mag_bias_x));
						mag.add(std::to_string(imu_mag_bias_y));
						mag.add(std::to_string(imu_mag_bias_z));
						mag.add(std::to_string(imu_mag_scale_x));
						mag.add(std::to_string(imu_mag_scale_y));
						mag.add(std::to_string(imu_mag_scale_z));
						root["ADV#"] = (uint8_t)advance;
					}
					else
					{
						root["S"] = 1;
						JsonArray cmd = root.createNestedArray("CMD!");
						cmd.add(std::to_string(cmd_temp[0]));
						cmd.add(std::to_string(cmd_temp[1]));
						if(gpo_write_update)
						{
							JsonArray gpo = root.createNestedArray("DO!");
							gpo.add(gpo_write_temp[0]);
							gpo.add(gpo_write_temp[1]);
							gpo.add(gpo_write_temp[2]);
							gpo.add(gpo_write_temp[3]);
							gpo.add(gpo_write_temp[4]);
							gpo.add(gpo_write_temp[5]);
							gpo_write_update = false;
						}
					}
					serializeJson(root, data);
					root["CRC"] = modbusCRC(reinterpret_cast<uint8_t*>(&data[0]),data.length(),0);
					data = "";
					serializeJson(root, data);
					if(debug)
					{
						// ROS_INFO_STREAM("Send: "<<data<<"\\r\\n");
						std_msgs::String debug_msg;
						debug_msg.data = "send: " + data;
						debug_publisher.publish(debug_msg);
					}
					data += "\r\n";
					ser->write(data);
					receive_timeout = ros::Time::now();
					one_time_send = false;
					receive_status  = false;
				}
				else
				{
					if((ros::Time::now() - receive_timeout).toSec() <= 0.5)
					{
						if(ser->available())
						{
							rev += ser->readline(ser->available(),"\n");
							if (rev.find("\n")!=std::string::npos)
							{
								std::string temp_str = rev;
								StaticJsonDocument<6000> root;
								DeserializationError error = deserializeJson(root, rev);
								if(!error)
								{
									if(!root["CRC"].isNull())
									{
										uint16_t tempCRC;
										tempCRC = root["CRC"].as<uint16_t>();
										root.remove("CRC");
										rev = "";
										serializeJson(root, rev);
										if( tempCRC == modbusCRC(reinterpret_cast<uint8_t*>(&rev[0]),rev.length(),0) )
										{
											// encoder twist angular x and angular z
											if(root["ENC?"].size() == 2)
											{
												geometry_msgs::TwistWithCovarianceStamped encoder_msg;
												encoder_msg.header.stamp = ros::Time::now();
												encoder_msg.header.frame_id = base_frame_id;
												encoder_msg.twist.twist.linear.x = root["ENC?"][0].as<double>();
												encoder_msg.twist.twist.angular.z = root["ENC?"][1].as<double>();
												encoder_msg.twist.covariance =	{	0.001,	0.000,	0.000,	0.000,	0.000,	0.000,
																					0.000,	0.001,	0.000,	0.000,	0.000,	0.000,
																					0.000,	0.000,	0.001,	0.000,	0.000,	0.000,
																					0.000,	0.000,	0.000,	0.001,	0.000,	0.000,
																					0.000,	0.000,	0.000,	0.000,	0.001,	0.000,
																					0.000,	0.000,	0.000,	0.000,	0.000,	0.030	};
												encoder_publisher.publish(encoder_msg);
											}

											// AHRS orientation, gyro and accelarometer
											if( (root["AHRS?"].size() == 4) && (root["IMU?"].size() == 9) )
											{
												if(root["IMU?"][5].as<double>()<=2.0 && root["IMU?"][5].as<double>()>=-2.0)
												{
													sensor_msgs::Imu imu_msg;
													imu_msg.header.stamp = ros::Time::now();
													imu_msg.header.frame_id = base_frame_id;
													imu_msg.orientation.x = root["AHRS?"][0].as<double>();//0
													imu_msg.orientation.y = root["AHRS?"][1].as<double>();//1
													imu_msg.orientation.z = root["AHRS?"][2].as<double>();//2
													imu_msg.orientation.w = root["AHRS?"][3].as<double>();//3
													imu_msg.orientation_covariance = {	2.6030820491461885e-7,	0.000,					0.000,
																						0.000,					2.6030820491461885e-7,	0.000,
																						0.000,					0.000,					0.000	};
													imu_msg.angular_velocity.x = root["IMU?"][3].as<double>();
													imu_msg.angular_velocity.y = root["IMU?"][4].as<double>();
													imu_msg.angular_velocity.z = root["IMU?"][5].as<double>();
													imu_msg.angular_velocity_covariance = {	2.5e-5,	0.000,	0.000,
																							0.000,	2.5e-5,	0.000,
																							0.000,	0.000,	2.5e-5	};
													imu_msg.linear_acceleration.x = root["IMU?"][0].as<double>();
													imu_msg.linear_acceleration.y = root["IMU?"][1].as<double>();
													imu_msg.linear_acceleration.z = root["IMU?"][2].as<double>();
													imu_msg.linear_acceleration_covariance = {	2.5e-5,	0.000,	0.000,
																								0.000,	2.5e-5,	0.000,
																								0.000,	0.000,	2.5e-5	};
													imu_publisher.publish(imu_msg);
												}
											}
											else
											{
												ROS_INFO_STREAM("mrc10100: imu->"<<rev);
											}

											// magnetic field strength
											if(root["IMU?"].size() == 9)
											{
												sensor_msgs::MagneticField magnetic_field_msg;
												magnetic_field_msg.header.stamp = ros::Time::now();
												magnetic_field_msg.header.frame_id = base_frame_id;
												magnetic_field_msg.magnetic_field.x = root["IMU?"][6].as<double>();
												magnetic_field_msg.magnetic_field.y = root["IMU?"][7].as<double>();
												magnetic_field_msg.magnetic_field.z = root["IMU?"][8].as<double>();
												magnetic_field_msg.magnetic_field_covariance = {	2.5e-5,	0.000,	0.000,
																									0.000,	2.5e-5,	0.000,
																									0.000,	0.000,	2.5e-5	};
												magnetic_field_publisher.publish(magnetic_field_msg);
											}

											// digital input status
											if(root["DI?"].size() == 9)
											{
												std_msgs::Int16MultiArray gpi_msg;
												gpi_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
												gpi_msg.layout.dim[0].size = root["DI?"].size();
												gpi_msg.layout.dim[0].stride = 1;
												gpi_msg.layout.dim[0].label = "gpi";
												for(int i=0; i<9; i++)
												{
													gpi_msg.data.push_back( root["DI?"][i].as<uint8_t>() );
												}

												gpi_publisher.publish(gpi_msg);
											}

											// digital output status
											if(root["DO?"].size() == 6)
											{
												std_msgs::Int16MultiArray gpo_read_msg;
												gpo_read_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
												gpo_read_msg.layout.dim[0].size = root["DO?"].size();
												gpo_read_msg.layout.dim[0].stride = 1;
												gpo_read_msg.layout.dim[0].label = "gpo";
												for(int i=0; i<6; i++)
												{
													gpo_read_msg.data.push_back( root["DO?"][i].as<uint8_t>() );
												}

												gpo_read_publisher.publish(gpo_read_msg);
											}

											// analog input reading
											if(root["ANA?"].size() == 2)
											{
												std_msgs::Float64MultiArray ana_msg;
												ana_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
												ana_msg.layout.dim[0].size = root["ANA?"].size();
												ana_msg.layout.dim[0].stride = 1;
												ana_msg.layout.dim[0].label = "analog";
												for(int i=0; i<2; i++)
												{
													ana_msg.data.push_back( root["ANA?"][i].as<double>() );
												}
												analog_publisher.publish(ana_msg);
											}

											// voltage and current measurement
											if(root["VNA?"].size() == 2)
											{
												std_msgs::Float64MultiArray vna_msg;
												vna_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
												vna_msg.layout.dim[0].size = root["VNA?"].size();
												vna_msg.layout.dim[0].stride = 1;
												vna_msg.layout.dim[0].label = "voltage_current";
												for(int i=0; i<2; i++)
												{
													vna_msg.data.push_back( root["VNA?"][i].as<double>() );
												}
												voltage_current_publisher.publish(vna_msg);
											}

											// encoder counter
											if(root["CNT?"].size() == 2)
											{
												std_msgs::Int32MultiArray cnt_msg;
												cnt_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
												cnt_msg.layout.dim[0].size = root["CNT?"].size();
												cnt_msg.layout.dim[0].stride = 1;
												cnt_msg.layout.dim[0].label = "count";
												for(int i=0; i<2; i++)
												{
													cnt_msg.data.push_back( root["CNT?"][i].as<int32_t>() );
												}
												count_publisher.publish(cnt_msg);
											}

											// error status
											if(root["E"].size() == 5)
											{
												status_temp[0] = 0;
												for(int i=0; i<5; i++)
												{
													status_temp[0] <<= 1;
													status_temp[0] = status_temp[0] | (root["E"][i].as<int32_t>() > 0 ? true : false);
												}
											}
											if(!success_receive)
											{
												success_receive  = true;
												ROS_INFO_STREAM("mrc10100: success receive data");
											}
											receive_status = true;
										}
										else
										{
											ROS_ERROR_STREAM("mrc10100: CRC didnt match-> "<<temp_str);
										}

									}
									else
									{
										ROS_ERROR_STREAM("mrc10100: CRC missing-> "<<temp_str);
									}
								}
								else
								{
									ROS_ERROR_STREAM("mrc10100: JSON parse fail-> "<<temp_str);
								}

								if(debug)
								{
									size_t index = 0;
									while(ros::ok())
									{
										index = temp_str.find("\r", index);
										if(index == std::string::npos) break;
										temp_str.replace(index, 1, "\\r");
										index += 1;
									}
									index = 0;
									while(ros::ok())
									{
										index = temp_str.find("\n", index);
										if(index == std::string::npos) break;
										temp_str.replace(index, 1, "\\n");
										index += 1;
									}
									// ROS_INFO_STREAM("Receive: "<<temp_str);
									std_msgs::String debug_msg;
									debug_msg.data = "receive: " + rev;
									debug_publisher.publish(debug_msg);
								}
								receive_status = true;
								rev = "";
							}
						}
					}
					else
					{
						ROS_ERROR_STREAM("mrc10100: fail to receive data");
						one_time_send = true;
						receive_status  = true;
						success_receive  = false;
						rev = "";
						ser->close();
					}
				}
			}
			catch(serial::PortNotOpenedException& e)
			{
				ROS_ERROR_STREAM("mrc10100: Serial disconnect, port fail to open");
				// ROS_ERROR_STREAM(e.what());
				ser->close();
			}
			catch(serial::SerialException& e)
			{
				ROS_ERROR_STREAM("mrc10100: Serial disconnect, port fail to read");
				// ROS_ERROR_STREAM(e.what());
				ser->close();
			}
			catch(serial::IOException& e)
			{
				ROS_ERROR_STREAM("mrc10100: Serial disconnect, port fail to ready");
				// ROS_ERROR_STREAM(e.what());
				ser->close();
			}
			catch(...)
			{
				ROS_ERROR_STREAM("mrc10100: unknown error to open serial port");
				ser->close();
			}
		}

		std_msgs::Int16MultiArray status_msg;
		status_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
		status_msg.layout.dim[0].size = 3;
		status_msg.layout.dim[0].stride = 1;
		status_msg.layout.dim[0].label = "openPortStatus,watchdogTrigger,deviceStatus";
		status_msg.data.push_back( ser->isOpen() ? 0 : -1);
		status_msg.data.push_back( success_receive ? 0 : -1);
		status_msg.data.push_back( status_temp[0] );
		status_publisher.publish(status_msg);

		loop_rate.sleep();
	}
	ser->flush();
	ser->close();
}

// CRC modbus
uint16_t Mrc10100::modbusCRC(uint8_t arrayByte[], uint8_t sizeByte, bool order = 0)
{
	uint16_t temp, temp2, flag;
	temp = 0xFFFF;
	for (unsigned char i = 0; i < sizeByte; i++)
	{
		temp = temp ^ arrayByte[i];
		for (unsigned char j = 1; j <= 8; j++)
		{
			flag = temp & 0x0001;
			temp >>=1;
			if (flag)
			temp ^= 0xA001;
		}
	}
	// Reverse byte order.
	if(!order)
	{
		temp2 = temp >> 8;
		temp = (temp << 8) | temp2;
		temp &= 0xFFFF;
		// the returned value is already swapped
		// crcLo byte is first & crcHi byte is last
	}
	return temp;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mrc10100");

	ros::AsyncSpinner spinner(0);
	spinner.start();

	Mrc10100 _Mrc10100;

	_Mrc10100.run();

	// dynamic_reconfigure::Server<mrc10100::mrc10100Config> server;
	// dynamic_reconfigure::Server<mrc10100::mrc10100Config>::CallbackType f;

	// f = boost::bind(&Mrc10100::reconfigureCallback,&_Mrc10100, _1, _2);
	// server.setCallback(f);

	// ros::spin();
	ros::waitForShutdown();

	return 0;
}
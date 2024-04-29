#include "ros/ros.h"
#include <ros/time.h>
#include <sensor_msgs/Imu.h>// imu msg
#include <iostream>

using namespace std;

class GyroAutocalibrate
{
	public:
		GyroAutocalibrate();
		void imuCallback(const sensor_msgs::Imu::ConstPtr& imu, std::string topic);

	private:
		ros::NodeHandle node_;
		ros::Subscriber imu_subscriber;
		ros::Publisher imu_publisher;
		ros::Publisher debug_publisher_high_rms;
		ros::Publisher debug_publisher_low_rms;

		string input_topic = "input";
		string output_topic = "output";

		ros::Time time_sampling = ros::Time::now();
		bool initial_data = true;
		bool initial_data_x = true;
		bool initial_data_y = true;
		bool initial_data_z = true;
		bool debug_data = false;

		double T_ = ros::Time::now().toSec();

		ros::Time initial_timeout_x, ideal_timeout_x;
		bool ideal_x = false;
		double Xgx_[2]={0, 0}, X3gx_[2]={0, 0}, X4gx_[2]={0, 0}, Y1gx_[2]={0, 0}, Y2gx_[2]={0, 0}, Y3gx_[2]={0, 0}, Y4gx_[2]={0, 0}, BIASgx_=0, initial_bias_x=0;
		double high_rms_threshold_gx=0.002, low_rms_threshold_gx=0.002;

		ros::Time initial_timeout_y, ideal_timeout_y;
		bool ideal_y = false;
		double Xgy_[2]={0, 0}, X3gy_[2]={0, 0}, X4gy_[2]={0, 0}, Y1gy_[2]={0, 0}, Y2gy_[2]={0, 0}, Y3gy_[2]={0, 0}, Y4gy_[2]={0, 0}, BIASgy_=0, initial_bias_y=0;
		double high_rms_threshold_gy=0.002, low_rms_threshold_gy=0.002;

		ros::Time initial_timeout_z, ideal_timeout_z;
		bool ideal_z = false;
		double Xgz_[2]={0, 0}, X3gz_[2]={0, 0}, X4gz_[2]={0, 0}, Y1gz_[2]={0, 0}, Y2gz_[2]={0, 0}, Y3gz_[2]={0, 0}, Y4gz_[2]={0, 0}, BIASgz_=0, initial_bias_z=0;
		double high_rms_threshold_gz=0.002, low_rms_threshold_gz=0.002;
};

GyroAutocalibrate::GyroAutocalibrate()
{
	ros::NodeHandle nh("~");

	//Setting class parameters
	if(!nh.getParam("input_topic", input_topic))
		input_topic = "input";
	if(!nh.getParam("output_topic", output_topic))
		output_topic = "output";
	if(!nh.getParam("debug", debug_data))
		debug_data = false;
	if(!nh.getParam("high_rms_threshold_gx", high_rms_threshold_gx))
		high_rms_threshold_gx = 0.002;
	if(!nh.getParam("low_rms_threshold_gx", low_rms_threshold_gx))
		low_rms_threshold_gx = 0.002;
	if(!nh.getParam("high_rms_threshold_gy", high_rms_threshold_gy))
		high_rms_threshold_gy = 0.002;
	if(!nh.getParam("low_rms_threshold_gy", low_rms_threshold_gy))
		low_rms_threshold_gy = 0.002;
	if(!nh.getParam("high_rms_threshold_gz", high_rms_threshold_gz))
		high_rms_threshold_gz = 0.002;
	if(!nh.getParam("low_rms_threshold_gz", low_rms_threshold_gz))
		low_rms_threshold_gz = 0.002;

	imu_subscriber = node_.subscribe<sensor_msgs::Imu> (input_topic.c_str(), 1, boost::bind(&GyroAutocalibrate::imuCallback,this, _1, input_topic));
	imu_publisher = node_.advertise<sensor_msgs::Imu> (output_topic.c_str(), 1, false);

	if(debug_data)
	{
		debug_publisher_high_rms = node_.advertise<sensor_msgs::Imu>("gyro/high_rms", 1);
		debug_publisher_low_rms = node_.advertise<sensor_msgs::Imu>("gyro/low_rms", 1);
	}
}

void GyroAutocalibrate::imuCallback(const sensor_msgs::Imu::ConstPtr& imu, std::string topic)
{
	sensor_msgs::ImuPtr output(new sensor_msgs::Imu());
	*output = *imu;

	// output->header.seq = imu->header.seq;
	// output->header.stamp = imu->header.stamp;  //fixes #265
	// output->header.frame_id = imu->header.frame_id;

	// output->orientation.x = imu->orientation.x;
	// output->orientation.y = imu->orientation.y;
	// output->orientation.z = imu->orientation.z;
	// output->orientation.w = imu->orientation.w;
	// output->orientation_covariance = imu->orientation_covariance;

	// output->angular_velocity.x = imu->angular_velocity.x;
	// output->angular_velocity.y = imu->angular_velocity.y;
	// output->angular_velocity.z = imu->angular_velocity.z;
	// output->angular_velocity_covariance = imu->angular_velocity_covariance;

	// output->linear_acceleration.x = imu->linear_acceleration.x;
	// output->linear_acceleration.y = imu->linear_acceleration.y;
	// output->linear_acceleration.z = imu->linear_acceleration.z;
	// output->linear_acceleration_covariance = imu->linear_acceleration_covariance;


	if(initial_data)
	{
		initial_data = false;
		time_sampling = ros::Time::now();

		initial_timeout_x = ros::Time::now();
		ideal_timeout_x = ros::Time::now();
		Xgx_[0] = imu->angular_velocity.x - initial_bias_x;
		Y1gx_[0] = Xgx_[0];
		Y2gx_[0] = Xgx_[0];
		X3gx_[0] = abs(Xgx_[0]);
		Y3gx_[0] = abs(Xgx_[0]);
		X4gx_[0] = abs(Xgx_[0]);
		Y4gx_[0] = abs(Xgx_[0]);

		initial_timeout_y = ros::Time::now();
		ideal_timeout_y = ros::Time::now();
		Xgy_[0] = imu->angular_velocity.y - initial_bias_y;
		Y1gy_[0] = Xgy_[0];
		Y2gy_[0] = Xgy_[0];
		X3gy_[0] = abs(Xgy_[0]);
		Y3gy_[0] = abs(Xgy_[0]);
		X4gy_[0] = abs(Xgy_[0]);
		Y4gy_[0] = abs(Xgy_[0]);

		initial_timeout_z = ros::Time::now();
		ideal_timeout_z = ros::Time::now();
		Xgz_[0] = imu->angular_velocity.z - initial_bias_z;
		Y1gz_[0] = Xgz_[0];
		Y2gz_[0] = Xgz_[0];
		X3gz_[0] = abs(Xgz_[0]);
		Y3gz_[0] = abs(Xgz_[0]);
		X4gz_[0] = abs(Xgz_[0]);
		Y4gz_[0] = abs(Xgz_[0]);
	}
	else
	{
		T_ = (ros::Time::now() - time_sampling).toSec();

		if(T_ > 0)
		{
			Xgx_[1] = Xgx_[0];
			Xgx_[0] = imu->angular_velocity.x - initial_bias_x;

			//lowpass
			Y1gx_[1] = Y1gx_[0];
			Y1gx_[0] = 2*M_PI*T_*Xgx_[1] + (1 - 2*M_PI*T_)*Y1gx_[1];

			//highpass
			Y2gx_[1] = Y2gx_[0];
			Y2gx_[0] = Xgx_[0] - Xgx_[1] + (1 - 2*M_PI*T_)*Y2gx_[1];

			//rms highpass
			X3gx_[1] = X3gx_[0];
			X3gx_[0] = abs(Y2gx_[0]);
			Y3gx_[1] = Y3gx_[0];
			Y3gx_[0] = 2*M_PI*T_*X3gx_[1] + (1 - 2*M_PI*T_)*Y3gx_[1];

			//rms lowpass
			X4gx_[1] = X4gx_[0];
			X4gx_[0] = abs(imu->angular_velocity.x - initial_bias_x);
			Y4gx_[1] = Y4gx_[0];
			Y4gx_[0] = 2*M_PI*T_*X4gx_[1] + (1 - 2*M_PI*T_)*Y4gx_[1];

			if( (Y3gx_[0]<=high_rms_threshold_gx) && (Y4gx_[0]<=low_rms_threshold_gx) )
			{
				if( ((ros::Time::now()-ideal_timeout_x).toSec()>=1) && !ideal_x )
				{
					initial_bias_x = Y1gx_[0] + initial_bias_x;
					// ideal_x = true;
					ideal_timeout_x = ros::Time::now();
				}
				// else if(ideal_x)
				// {
				// 	BIASgx_ = Y1gx_[0];
				// 	ideal_timeout_x = ros::Time::now();
				// }
			}
			else if( (Y3gx_[0]>high_rms_threshold_gx) )
			{
				ideal_x = false;
				ideal_timeout_x = ros::Time::now();
			}

			if( (Y3gx_[0]<=high_rms_threshold_gx) && (initial_data_x) )
			{
				if((ros::Time::now()-initial_timeout_x).toSec()>=1)
				{
					initial_bias_x = Y1gx_[0] + initial_bias_x;
					initial_data_x = false;
				}
			}
			else
			{
				initial_timeout_x = ros::Time::now();
			}

			Xgy_[1] = Xgy_[0];
			Xgy_[0] = imu->angular_velocity.y - initial_bias_y;

			//lowpass
			Y1gy_[1] = Y1gy_[0];
			Y1gy_[0] = 2*M_PI*T_*Xgy_[1] + (1 - 2*M_PI*T_)*Y1gy_[1];

			//highpass
			Y2gy_[1] = Y2gy_[0];
			Y2gy_[0] = Xgy_[0] - Xgy_[1] + (1 - 2*M_PI*T_)*Y2gy_[1];

			//rms highpass
			X3gy_[1] = X3gy_[0];
			X3gy_[0] = abs(Y2gy_[0]);
			Y3gy_[1] = Y3gy_[0];
			Y3gy_[0] = 2*M_PI*T_*X3gy_[1] + (1 - 2*M_PI*T_)*Y3gy_[1];

			//rms lowpass
			X4gy_[1] = X4gy_[0];
			X4gy_[0] = abs(imu->angular_velocity.y - initial_bias_y);
			Y4gy_[1] = Y4gy_[0];
			Y4gy_[0] = 2*M_PI*T_*X4gy_[1] + (1 - 2*M_PI*T_)*Y4gy_[1];

			if( (Y3gy_[0]<=high_rms_threshold_gy) && (Y4gy_[0]<=low_rms_threshold_gy) )
			{
				if( ((ros::Time::now()-ideal_timeout_y).toSec()>=1) && !ideal_y )
				{
					initial_bias_y = Y1gz_[0] + initial_bias_y;
					// ideal_y = true;
					ideal_timeout_y = ros::Time::now();
				}
				// else if(ideal_y)
				// {
				// 	BIASgy_ = Y1gy_[0];
				// 	ideal_timeout_y = ros::Time::now();
				// }
			}
			else if( (Y3gy_[0]>high_rms_threshold_gy) )
			{
				ideal_y = false;
				ideal_timeout_y = ros::Time::now();
			}

			if( (Y3gy_[0]<=high_rms_threshold_gy) && (initial_data_y) )
			{
				if((ros::Time::now()-initial_timeout_y).toSec()>=1)
				{
					initial_bias_y = Y1gx_[0] + initial_bias_y;
					initial_data_y = false;
				}
			}
			else
			{
				initial_timeout_y = ros::Time::now();
			}

			Xgz_[1] = Xgz_[0];
			Xgz_[0] = imu->angular_velocity.z - initial_bias_z;

			//lowpass
			Y1gz_[1] = Y1gz_[0];
			Y1gz_[0] = 2*M_PI*T_*Xgz_[1] + (1 - 2*M_PI*T_)*Y1gz_[1];

			//highpass
			Y2gz_[1] = Y2gz_[0];
			Y2gz_[0] = Xgz_[0] - Xgz_[1] + (1 - 2*M_PI*T_)*Y2gz_[1];

			//rms highpass
			X3gz_[1] = X3gz_[0];
			X3gz_[0] = abs(Y2gz_[0]);
			Y3gz_[1] = Y3gz_[0];
			Y3gz_[0] = 2*M_PI*T_*X3gz_[1] + (1 - 2*M_PI*T_)*Y3gz_[1];

			//rms lowpass
			X4gz_[1] = X4gz_[0];
			X4gz_[0] = abs(imu->angular_velocity.z - initial_bias_z);
			Y4gz_[1] = Y4gz_[0];
			Y4gz_[0] = 2*M_PI*T_*X4gz_[1] + (1 - 2*M_PI*T_)*Y4gz_[1];

			if( (Y3gz_[0]<=high_rms_threshold_gz) && (Y4gz_[0]<=low_rms_threshold_gz) )
			{
				if( ((ros::Time::now()-ideal_timeout_z).toSec()>=1) && !ideal_z )
				{
					initial_bias_z = Y1gz_[0] + initial_bias_z;
					// ideal_z = true;
					ideal_timeout_z = ros::Time::now();
				}
				// else if(ideal_z)
				// {
				// 	BIASgz_ = Y1gz_[0];
				// 	ideal_timeout_z = ros::Time::now();
				// }
			}
			else if( (Y3gz_[0]>high_rms_threshold_gz) )
			{
				ideal_z = false;
				ideal_timeout_z = ros::Time::now();
			}

			if( (Y3gz_[0]<=high_rms_threshold_gz) && (initial_data_z) )
			{
				if((ros::Time::now()-initial_timeout_z).toSec()>=1)
				{
					initial_bias_z = Y1gz_[0] + initial_bias_z;
					initial_data_z = false;
				}
			}
			else
			{
				initial_timeout_z = ros::Time::now();
			}

			output->angular_velocity.x = Xgx_[0] - BIASgx_;
			output->angular_velocity.y = Xgy_[0] - BIASgy_;
			output->angular_velocity.z = Xgz_[0] - BIASgz_;

		}
		time_sampling = ros::Time::now();
	}
	imu_publisher.publish(output);

	if(debug_data)
	{
		sensor_msgs::ImuPtr debug_output_low_rms(new sensor_msgs::Imu());
		sensor_msgs::ImuPtr debug_output_high_rms(new sensor_msgs::Imu());

		*debug_output_low_rms = *imu;
		*debug_output_high_rms = *imu;

		debug_output_low_rms->angular_velocity.x = Y4gx_[0];
		debug_output_low_rms->angular_velocity.y = Y4gy_[0];
		debug_output_low_rms->angular_velocity.z = Y4gz_[0];
		debug_output_high_rms->angular_velocity.x = Y3gx_[0];
		debug_output_high_rms->angular_velocity.y = Y3gy_[0];
		debug_output_high_rms->angular_velocity.z = Y3gz_[0];

		debug_publisher_low_rms.publish(debug_output_low_rms);
		debug_publisher_high_rms.publish(debug_output_high_rms);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gyro_autocalibrate");

	GyroAutocalibrate _Gyro_autocalibrate;

	ros::spin();

	return 0;
}

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <ros/ros.h>
#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxConfigureControl.h>

#include <com/sbapi.h>
#include <CoaxVisionControl.h>

CoaxVisionControl::CoaxVisionControl(ros::NodeHandle &node):
	LOW_POWER_DETECTED(false),
	coax_nav_mode(0),
	battery_voltage(12.22),
	imu_y(0.0),
	imu_r(0.0),
	imu_p(0.0),
	range_al(0.0),
	rc_th(0.0),
	rc_y(0.0),
	rc_r(0.0),
	rc_p(0.0),
	rc_trim_th(0.0),
	rc_trim_y(0.0),
	rc_trim_r(0.0),
	rc_trim_p(0.0),
	img_th(0.0),
	img_y(0.0),
	img_r(0.0),
	img_p(0.0),
	gyro_ch1(0.0),
	gyro_ch2(0.0),
	gyro_ch3(0.0),
	accel_x(0.0),
	accel_y(0.0),
	accel_z(0.0)		 
{
	reach_nav_state = node.serviceClient<coax_msgs::CoaxReachNavState>("reach_nav_state");
	configure_comm = node.serviceClient<coax_msgs::CoaxConfigureComm>("configure_comm");
	configure_control = node.serviceClient<coax_msgs::CoaxConfigureControl>("configure_control");
	set_timeout = node.serviceClient<coax_msgs::CoaxSetTimeout>("set_timeout");

	coax_state_sub = node.subscribe("state",1, &CoaxVisionControl::coaxStateCallback, this);

	raw_control_pub = node.advertise<coax_msgs::CoaxRawControl>("rawcontrol",1);

}

CoaxVisionControl::~CoaxVisionControl()
{
}

//===================
// Service Clients
//===================
	
bool CoaxVisionControl::reachNavState(int des_state, float timeout)
{
	coax_msgs::CoaxReachNavState srv;
	srv.request.desiredState = des_state;
	srv.request.timeout = timeout;
	reach_nav_state.call(srv);
	
	return 0;
}

bool CoaxVisionControl::configureComm(int frequency, int contents)
{
	coax_msgs::CoaxConfigureComm srv;
	srv.request.frequency = frequency;
	srv.request.contents = contents;
	configure_comm.call(srv);
	
	return 0;
}

bool CoaxVisionControl::configureControl(unsigned int rollMode, unsigned int pitchMode, unsigned int yawMode, unsigned int altitudeMode)
{
	coax_msgs::CoaxConfigureControl srv;
	srv.request.rollMode = rollMode;
	srv.request.pitchMode = pitchMode;
	srv.request.yawMode = yawMode;
	srv.request.altitudeMode = altitudeMode;
	configure_control.call(srv);
	//ROS_INFO("called configure control");
	return 0;
}

bool CoaxVisionControl::setTimeout(unsigned int control_timeout_ms, unsigned int watchdog_timeout_ms)
{
	coax_msgs::CoaxSetTimeout srv;
	srv.request.control_timeout_ms = control_timeout_ms;
	srv.request.watchdog_timeout_ms = watchdog_timeout_ms;
	set_timeout.call(srv);
	
	return 0;
}

//==============
// Subscriber
//==============

void CoaxVisionControl::coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message)
{
	battery_voltage = 0.8817*message->battery + 1.5299;
	coax_nav_mode = message->mode.navigation;
	
	if ((battery_voltage < 10.80) && !LOW_POWER_DETECTED){
		ROS_INFO("Battery Low!!! (%fV) Landing initialized",battery_voltage);
		LOW_POWER_DETECTED = true;
	}

	imu_y = message->yaw;
	imu_r = message->roll;
	imu_p = message->pitch;

	range_al = message->zfiltered;

	rc_th = message->rcChannel[0];
	rc_y = message->rcChannel[2];
	rc_r = message->rcChannel[4];
	rc_p = message->rcChannel[6];
	rc_trim_th = message->rcChannel[1];
	rc_trim_y = message->rcChannel[3];
	rc_trim_r = message->rcChannel[5];
	rc_trim_p = message->rcChannel[7];

	gyro_ch1 = message->gyro[0];
	gyro_ch2 = message->gyro[1];
	gyro_ch3 = message->gyro[2];

	accel_x = message->accel[0];
	accel_y = message->accel[1];
	accel_z = message->accel[2];

}

void CoaxVisionControl::rawControlPublisher(unsigned int rate)
{
	ros::Rate loop_rate(rate);

	coax_msgs::CoaxRawControl raw_control;

	while(ros::ok())
	{
		raw_control.motor1 = 0;
		raw_control.motor2 = 0;
		raw_control.servo1 = 0;
		raw_control.servo2 = 0;
		//raw_control_pub.publish(raw_control);

		ros::spinOnce();
		loop_rate.sleep();
	}
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "CoaxVisionControl");
	ros::NodeHandle nh("/coax_vision_control");
  ImageProc vision(nh);  
	CoaxVisionControl control(nh);

	// make sure coax_server has enough time to start
	ros::Duration(1.5).sleep(); 

	control.configureComm(100, SBS_MODES | SBS_BATTERY | SBS_GYRO | SBS_ACCEL | SBS_CHANNELS | SBS_RPY | SBS_HRANGES);
	// control.setTimeout(500, 5000);

	control.configureControl(SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL);
	control.setTimeout(500, 5000);

	ROS_INFO("Initially Setup comm and control");

	int frequency = 100;
	control.rawControlPublisher(frequency);

  ros::spin();
  return 0;
}




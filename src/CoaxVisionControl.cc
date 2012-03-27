#include <iostream>
#include <cmath>
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

CoaxVisionControl::CoaxVisionControl(ros::NodeHandle &node, ImageProc & cImageProc)
:reach_nav_state(node.serviceClient<coax_msgs::CoaxReachNavState>("reach_nav_state"))
,configure_comm(node.serviceClient<coax_msgs::CoaxConfigureComm>("configure_comm"))
,configure_control(node.serviceClient<coax_msgs::CoaxConfigureControl>("configure_control"))
,set_timeout(node.serviceClient<coax_msgs::CoaxSetTimeout>("set_timeout"))

,coax_state_sub(node.subscribe("state",1, &CoaxVisionControl::coaxStateCallback, this))

,raw_control_pub(node.advertise<coax_msgs::CoaxRawControl>("rawcontrol",1))
,vision_control_pub(node.advertise<coax_msgs::CoaxControl>("visioncontrol",1))
,image(cImageProc)
,LOW_POWER_DETECTED(false)
,CONTROL_MODE(CONTROL_LANDED)
,FIRST_START(false)
,FIRST_STATE(true)
,FIRST_LANDING(false)
,FIRST_HOVER(false)
,INIT_DESIRE(false)
,INIT_IMU(false)
,coax_nav_mode(0)
,coax_control_mode(0)
,coax_state_age(0)
,raw_control_age(0)
,init_count(0)
,init_imu_count(0)
,rotor_ready_count(-1)
,last_state_time(0.0)
,battery_voltage(12.22)
,init_imu_x(0.0),init_imu_y(0.0),init_imu_z(0.0)
,imu_y(0.0),imu_r(0.0),imu_p(0.0),imu_al(0.0)
,range_al(0.0)
,rc_th(0.0),rc_y(0.0),rc_r(0.0),rc_p(0.0)
,rc_trim_th(0.0),rc_trim_y(0.104),rc_trim_r(0.054),rc_trim_p(0.036)
,img_th(0.0),img_y(0.0),img_r(0.0),img_p(0.0)
,gyro_ch1(0.0),gyro_ch2(0.0),gyro_ch3(0.0)
,accel_x(0.0),accel_y(0.0),accel_z(0.0)
,gyro_ch1_init(0.0),gyro_ch2_init(0.0),gyro_ch3_init(0.0)
,accel_x_init(0.0),accel_y_init(0.0),accel_z_init(0.0)
,pos_z(0.0),vel_z(0.0)
,motor_up(0),motor_lo(0)
,servo_roll(0),servo_pitch(0)
,roll_trim(0),pitch_trim(0)
,motor1_des(0.0),motor2_des(0.0),servo1_des(0.0),servo2_des(0.0)
,yaw_des(0.0),yaw_rate_des(0.0)
,roll_des(0.0),roll_rate_des(0.0)
,pitch_des(0.0),pitch_rate_des(0.0)
,altitude_des(0.0)
,z(0.0),z_v(0.0)
{
	set_nav_mode.push_back(node.advertiseService("set_nav_mode", &CoaxVisionControl::setNavMode, this));
	set_control_mode.push_back(node.advertiseService("set_control_mode", &CoaxVisionControl::setControlMode, this));
	loadParams(node);
}

CoaxVisionControl::~CoaxVisionControl() {}

void CoaxVisionControl::loadParams(ros::NodeHandle &n) {
	n.getParam("motorconst/const1",motor_const1);
	n.getParam("motorconst/const2",motor_const2);
	n.getParam("rollconst/const",servo1_const);
	n.getParam("pitchconst/const",servo2_const);
	n.getParam("yawcoef/coef1",yaw_coef1);
	n.getParam("yawcoef/coef2",yaw_coef2);
	n.getParam("throttlecoef/coef1",thr_coef1);
	n.getParam("throttlecoef/coef2",thr_coef2);
	n.getParam("rollrccoef/coef",r_rc_coef);
	n.getParam("pitchrccoef/coef",p_rc_coef);
	n.getParam("yawcontrol/proportional",kp_yaw);
	n.getParam("yawcontrol/differential",kd_yaw);
	n.getParam("rollcontrol/proportional",kp_roll);
	n.getParam("rollcontrol/differential",kd_roll);
	n.getParam("pitchcontrol/proportional",kp_pitch);
	n.getParam("pitchcontrol/differential",kd_pitch);
	n.getParam("altitude/base",range_base);
	n.getParam("altitudecontrol/proportional",kp_altitude);
	n.getParam("imageyawcontrol/proportional",kp_imgyaw);
	n.getParam("imagerollcontrol/proportional",kp_imgroll);

	imu_al = range_base;
}

//===================
// Service Clients
//===================
	
bool CoaxVisionControl::reachNavState(int des_state, float timeout) {
	coax_msgs::CoaxReachNavState srv;
	srv.request.desiredState = des_state;
	srv.request.timeout = timeout;
	reach_nav_state.call(srv);	
	return 0;
}

bool CoaxVisionControl::configureComm(int frequency, int contents) {
	coax_msgs::CoaxConfigureComm srv;
	srv.request.frequency = frequency;
	srv.request.contents = contents;
	configure_comm.call(srv);
	
	return 0;
}

bool CoaxVisionControl::configureControl(size_t rollMode, size_t pitchMode, size_t yawMode, size_t altitudeMode) {
	coax_msgs::CoaxConfigureControl srv;
	srv.request.rollMode = rollMode;
	srv.request.pitchMode = pitchMode;
	srv.request.yawMode = yawMode;
	srv.request.altitudeMode = altitudeMode;
	configure_control.call(srv);
	//ROS_INFO("called configure control");
	return 0;
}

bool CoaxVisionControl::setTimeout(size_t control_timeout_ms, size_t watchdog_timeout_ms) {
	coax_msgs::CoaxSetTimeout srv;
	srv.request.control_timeout_ms = control_timeout_ms;
	srv.request.watchdog_timeout_ms = watchdog_timeout_ms;
	set_timeout.call(srv);
	
	return 0;
}
//==============
//ServiceServer
//==============
bool CoaxVisionControl::setNavMode(coax_vision::SetNavMode::Request &req, coax_vision::SetNavMode::Response &out) {
	out.result = 0;
	
	switch (req.mode) 
	{
		case SB_NAV_STOP:
			break;
		case SB_NAV_IDLE:
			break;
	}
	return 0;
}

bool CoaxVisionControl::setControlMode(coax_vision::SetControlMode::Request &req, coax_vision::SetControlMode::Response &out) {
	out.result = 0;
	
	switch (req.mode) 
	{
		case 1:
			if (CONTROL_MODE != CONTROL_LANDED){
				ROS_INFO("Start can only be executed from mode CONTROL_LANDED");
				out.result = -1;
				break;
			}

			if (battery_voltage < 10.5) {
				ROS_INFO("Battery Low!!! (%f V) Start denied",battery_voltage);
				LOW_POWER_DETECTED = true;
				out.result = -1;
				break;
			}

			if (coax_nav_mode != SB_NAV_RAW) {
				if (coax_nav_mode != SB_NAV_STOP) {
					reachNavState(SB_NAV_STOP, 0.5);
					ros::Duration(0.5).sleep(); // make sure CoaX is in SB_NAV_STOP mode
				}
				reachNavState(SB_NAV_RAW, 0.5);
			}
			// switch to start procedure
			INIT_DESIRE = false;
			init_count = 0;
			rotor_ready_count = 0;
			CONTROL_MODE = CONTROL_START;
			motor1_des = 0;
			motor2_des = 0;
			servo1_des = 0;
			servo2_des = 0;
			break;

		case 9:
			motor1_des = 0;
			motor2_des = 0;
			servo1_des = 0;
			servo2_des = 0;
			roll_trim = 0;
			pitch_trim = 0;
			
			reachNavState(SB_NAV_STOP, 0.5);
			
			rotor_ready_count = -1;
			CONTROL_MODE = CONTROL_LANDED;
			break;
		
		default:
			ROS_INFO("Non existent control mode request!");
			out.result = -1;		
	}
	return true;
}


//==============
// Subscriber
//==============

void CoaxVisionControl::coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message) {
	double cur_state_time = 0;
	double time_duration = 0;
	battery_voltage = 0.8817*message->battery + 1.5299;
	coax_nav_mode = message->mode.navigation;
	
	if (FIRST_STATE) {
		last_state_time = ros::Time::now().toSec();
		FIRST_STATE = false;
		ROS_INFO("First Time Stamp: %f",last_state_time);
		return;
	}	

	if ((battery_voltage < 10.30) && !LOW_POWER_DETECTED){
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

	gyro_ch1 = message->gyro[0];
	gyro_ch2 = message->gyro[1];
	gyro_ch3 = message->gyro[2];

	accel_x = message->accel[0];// - init_imu_x;
	accel_y = message->accel[1];// - init_imu_y;
	accel_z = message->accel[2];// - init_imu_z;
	return;
}

bool CoaxVisionControl::setRawControl(double motor1, double motor2, double servo1, double servo2) {
	coax_msgs::CoaxRawControl raw_control;

	motor_up = motor1;
	motor_lo = motor2;
	servo_roll = servo1;
	servo_pitch = servo2;

	if (motor1 > 1) 
		motor_up = 1;
	else if (motor1 < 0) 
		motor_up = 0;
	else 
		motor_up = motor1;

	if (motor2 > 1) 
		motor_lo = 1;
	else if (motor2 < 0) 
		motor_lo = 0;
	else 
		motor_lo = motor2;

	if (servo1 > 1)
		servo_roll = 1;
	else if (servo1 < -1)
		servo_roll = -1;
	else
		servo_roll = servo1;

	if (servo2 > 1)
		servo_pitch = 1;
	else if (servo2 < -1)
		servo_pitch = -1;
	else
		servo_pitch = servo2;
	
	raw_control.motor1 = motor_up;
	raw_control.motor2 = motor_lo;
	raw_control.servo1 = servo_roll;
	raw_control.servo2 = servo_pitch;

	raw_control_pub.publish(raw_control);

	return 1;
}

bool CoaxVisionControl::rotorReady(void) {
	if (rotor_ready_count < 0) return false;
	if (rotor_ready_count <= 300) 
		rotor_ready_count++;
	if (rotor_ready_count < 150) {
		motor1_des = rotor_ready_count / 150 * motor_const1;
		motor2_des = 0;
		return false;
	}
	else if (rotor_ready_count < 300) {
		motor1_des = motor_const1;
		motor2_des = (rotor_ready_count - 150) / 150 * motor_const2;
		return false;
	}
	return true;	
}

void CoaxVisionControl::stabilizationControl(void) {
	double Dyaw,Dyaw_rate,yaw_control;
	double Droll,Droll_rate,roll_control;
	double Dpitch,Dpitch_rate,pitch_control;
	double altitude_control,Daltitude;

	altitude_des = range_base + thr_coef1 * rc_th;
	if (range_al < 0.25)
		Daltitude = 0;
	else
		Daltitude = range_al - altitude_des;
	altitude_control = altitude_des + kp_altitude * Daltitude;
//	ROS_INFO("range %f Daltitude %f",range_al,Daltitude);
	// yaw error and ctrl
	yaw_des += yaw_coef1*(rc_y+rc_trim_y);
	ROS_INFO("desired yaw %f", yaw_des);
	Dyaw = imu_y - yaw_des;
	Dyaw_rate = gyro_ch3 - yaw_rate_des; 
	yaw_control = kp_yaw * Dyaw + kd_yaw * Dyaw_rate; 
//	ROS_INFO("rc yaw %f", yaw_coef1*(rc_y+rc_trim_y));
	// roll error and ctrl
	Droll = imu_r - roll_des;
	Droll_rate = gyro_ch1 - roll_rate_des;
	roll_control = kp_roll * Droll + kd_roll * Droll_rate;
	// pitch error and ctrl
	Dpitch = imu_p - pitch_des;
	Dpitch_rate = gyro_ch2 - pitch_rate_des;
	pitch_control = kp_pitch * Dpitch + kd_pitch * Dpitch_rate;
	// desired motor & servo output
	motor1_des = motor_const1 - yaw_control + altitude_control;
	motor2_des = motor_const2 + yaw_control + altitude_control;
	servo1_des = servo1_const + r_rc_coef * (rc_r+rc_trim_r) + roll_control;
	servo2_des = servo2_const - p_rc_coef * (rc_p+rc_trim_p)  + pitch_control;
}

void CoaxVisionControl::visionControl(void) {
	double DyawIMG, yawIMG_control;
	double centerIMG = 30;
	SymAxis Axis;
	if (image.SortedAxis.size() == 0)
		return;
//	if (image.PeakAxis.size() != 0) {
//		size_t	peakmid = image.PeakAxis.size()/2;
//		std::cout << "first filter" << std::endl;
//		deque<SymAxis> Peak2;
//		for (size_t cnt = 1; cnt < image.PeakAxis.size()-1; cnt++) {
//			if ((image.PeakAxis[cnt].value > image.PeakAxis[cnt-1].value) &&
//					(image.PeakAxis[cnt].value > image.PeakAxis[cnt+1].value)) {
//						std::cout << image.PeakAxis[cnt].axis << ' ';
//						Peak2.push_back(image.PeakAxis[cnt]);
//			}
//		}
//		std::cout << std::endl;
//		if (Peak2.size()>0) {
//			for (size_t cnt = 1; cnt < Peak2.size()-1; cnt++) {
//				if ((Peak2[cnt].value > Peak2[cnt-1].value) &&
//						(Peak2[cnt].value > Peak2[cnt+1].value)) {
//							std::cout << Peak2[cnt].axis << ' ';
//				}
//			}
//		}
//		std::cout << std::endl;
//		std::cout << image.PeakAxis[peakmid-2].axis << ' '; // << image.PeakAxis[peakmid-2].value << ' ';
//		std::cout << image.PeakAxis[peakmid-1].axis << ' '; // << image.PeakAxis[peakmid-1].value << ' ';
//		std::cout << image.PeakAxis[peakmid].axis << ' '; // << image.PeakAxis[peakmid].value << ' ';
//		std::cout << image.PeakAxis[peakmid+1].axis	<< ' '; // << image.PeakAxis[peakmid+1].value << ' ';
//		std::cout << image.PeakAxis[peakmid+2].axis << std::endl; // ' ' << image.PeakAxis[peakmid+2].value << std::endl;	
//		ROS_INFO("Find %d peaks",image.PeakAxis.size());
//	}

	Axis = image.SortedAxis.front();
	DyawIMG = Axis.axis - centerIMG;
	yawIMG_control = kp_imgyaw * DyawIMG;
	motor1_des += yawIMG_control;
	motor2_des -= yawIMG_control; 
	servo1_des += kp_imgroll * image.shift;
//		ROS_INFO("Current Symmetric Axis: %d",Axis.axis);
}

void CoaxVisionControl::imuAnalysis(void) {
//		ROS_INFO("acc Z %f", accel_z + gravity);
	double currentTime = ros::Time::now().toSec();
	double deltaT = currentTime - last_state_time;
//	ROS_INFO("Delta Time %f", deltaT);
	last_state_time = currentTime;

	double curAccZ = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
	curAccZ -= gravity;	
	//curAccZ = (abs(curAccZ)>0.1)? curAccZ : 0;
//	ROS_INFO("curAccZ: %f", curAccZ);
	z = z + deltaT * z_v + 0.5 * deltaT * deltaT * curAccZ;
	z_v = z_v + deltaT * curAccZ; 	
	ROS_INFO("AccZ %f Z: %f, Z_V: %f",curAccZ,z, z_v);
	
}

void CoaxVisionControl::controlPublisher(size_t rate) {
	ros::Rate loop_rate(rate);

	double sum_Yaw_desire = 0;


	while(ros::ok()) {
		int init_iters = 1000;
		if ((init_count<init_iters)) {
			sum_Yaw_desire = sum_Yaw_desire + imu_y;
			gyro_ch1_init = gyro_ch1_init + gyro_ch1;
			gyro_ch2_init = gyro_ch2_init + gyro_ch2;
			gyro_ch3_init = gyro_ch3_init + gyro_ch3;
			accel_x_init = accel_x_init + accel_x;
			accel_y_init = accel_y_init + accel_y;
			accel_z_init = accel_z_init + accel_z;
			init_count ++;
		}
		else if (!INIT_DESIRE) {
			yaw_des = sum_Yaw_desire / init_iters;		
			ROS_INFO("Initiated Desired Yaw %f",yaw_des);
			gyro_ch1_init /= init_iters;
			gyro_ch2_init /= init_iters;
			gyro_ch3_init /= init_iters;
			accel_x_init /= init_iters;
			accel_y_init /= init_iters;
			accel_z_init /= init_iters;
			gravity = sqrt(pow(accel_x_init,2)+pow(accel_y_init,2)+pow(accel_z_init,2));
			last_state_time = ros::Time::now().toSec();
			ROS_INFO("Init Gyro Value: %f %f %f", gyro_ch1_init, gyro_ch2_init, gyro_ch3_init);
			ROS_INFO("Init Acce Value: %f %f %f with Gravity %f", 
								accel_x_init, accel_y_init, accel_z_init, gravity);
			ROS_INFO("Current Battery Voltage %f",battery_voltage);
			sum_Yaw_desire = 0;
			INIT_DESIRE = true;
		}

		if (INIT_DESIRE) {
			imuAnalysis();
		}
		
		if (INIT_DESIRE && rotorReady()) {
			stabilizationControl();
//			visionControl();
		}
		setRawControl(motor1_des,motor2_des,servo1_des,servo2_des);



		ros::spinOnce();
		loop_rate.sleep();
	}
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "CoaxVisionControl");
	ros::NodeHandle nh("~");
  ImageProc vision(nh);  
	CoaxVisionControl control(nh,vision);

	// make sure coax_server has enough time to start
	ros::Duration(1.5).sleep(); 

	control.configureComm(100, SBS_MODES | SBS_BATTERY | SBS_GYRO | SBS_ACCEL | SBS_CHANNELS | SBS_RPY | SBS_ALTITUDE_ALL);
	// control.setTimeout(500, 5000);

	control.configureControl(SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL);
	control.setTimeout(500, 5000);

	ROS_INFO("Initially Setup comm and control");

	int frequency = 100;
	control.controlPublisher(frequency);

  ros::spin();
  return 0;
}




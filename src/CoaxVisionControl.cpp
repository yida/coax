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



CoaxVisionControl::CoaxVisionControl(ros::NodeHandle &node)
:VisionFeedback(node), KF()

,reach_nav_state(node.serviceClient<coax_msgs::CoaxReachNavState>("reach_nav_state"))
,configure_comm(node.serviceClient<coax_msgs::CoaxConfigureComm>("configure_comm"))
,configure_control(node.serviceClient<coax_msgs::CoaxConfigureControl>("configure_control"))
,set_timeout(node.serviceClient<coax_msgs::CoaxSetTimeout>("set_timeout"))

,coax_state_sub(node.subscribe("state",1, &CoaxVisionControl::StateCallback, this))

,raw_control_pub(node.advertise<coax_msgs::CoaxRawControl>("rawcontrol",1))
,vision_control_pub(node.advertise<coax_msgs::CoaxControl>("visioncontrol",1))
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

,rpy(0,0,0), accel(0,0,0), gyro(0,0,0), rpyt_rc(0,0,0,0), rpyt_rc_trim(0,0,0,0)

,range_al(0.0)
,pos_z(0.0),vel_z(0.0)
,motor_up(0),motor_lo(0)
,servo_roll(0),servo_pitch(0)
,roll_trim(0),pitch_trim(0)
,motor1_des(0.0),motor2_des(0.0),servo1_des(0.0),servo2_des(0.0)
,yaw_des(0.0),yaw_rate_des(0.0)
,roll_des(0.0),roll_rate_des(0.0)
,pitch_des(0.0),pitch_rate_des(0.0)
,altitude_des(0.0)
{
	set_nav_mode.push_back(node.advertiseService("set_nav_mode", &CoaxVisionControl::setNavMode, this));
	set_control_mode.push_back(node.advertiseService("set_control_mode", &CoaxVisionControl::setControlMode, this));

}

CoaxVisionControl::~CoaxVisionControl() {}

void CoaxVisionControl::loadParams(ros::NodeHandle &n) {
	n.getParam("motorconst/const1",pm.motor_const1);
	n.getParam("motorconst/const2",pm.motor_const2);
	n.getParam("rollconst/const",pm.servo1_const);
	n.getParam("pitchconst/const",pm.servo2_const);
	n.getParam("yawcoef/coef1",pm.yaw_coef1);
	n.getParam("yawcoef/coef2",pm.yaw_coef2);
	n.getParam("throttlecoef/coef1",pm.thr_coef1);
	n.getParam("throttlecoef/coef2",pm.thr_coef2);
	n.getParam("rollrccoef/coef",pm.r_rc_coef);
	n.getParam("pitchrccoef/coef",pm.p_rc_coef);
	n.getParam("yawcontrol/proportional",pm.kp_yaw);
	n.getParam("yawcontrol/differential",pm.kd_yaw);
	n.getParam("rollcontrol/proportional",pm.kp_roll);
	n.getParam("rollcontrol/differential",pm.kd_roll);
	n.getParam("pitchcontrol/proportional",pm.kp_pitch);
	n.getParam("pitchcontrol/differential",pm.kd_pitch);
	n.getParam("altitude/base",pm.range_base);

	n.getParam("altitudecontrol/proportional",pm.kp_altitude);
	n.getParam("altitudecontrol/differential",pm.kd_altitude);
	n.getParam("altitudecontrol/integration",pm.ki_altitude);

	n.getParam("imageyawcontrol/proportional",pm.kp_imgyaw);
	n.getParam("imagerollcontrol/proportional",pm.kp_imgroll);

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

void CoaxVisionControl::StateCallback(const coax_msgs::CoaxState::ConstPtr & msg) {
	static int initTime = 200;
	static int initCounter = 0;

	static Eigen::Vector3f init_acc(0,0,0);
	static Eigen::Vector3f init_gyr(0,0,0);

	battery_voltage = 0.8817*msg->battery + 1.5299;
	coax_nav_mode = msg->mode.navigation;

	// Read from State
	rpy << msg->roll, msg->pitch, msg->yaw; 
//	std::cout << "RPY: \n" << rpy << std::endl;
	accel << msg->accel[0], msg->accel[1], msg->accel[2];
	gyro << msg->gyro[0], msg->gyro[1], msg->gyro[2];
	rpyt_rc << msg->rcChannel[4], msg->rcChannel[6], msg->rcChannel[2], msg->rcChannel[0];
	rpyt_rc_trim << msg->rcChannel[5], msg->rcChannel[7], msg->rcChannel[3], msg->rcChannel[1];
	range_al = msg->zfiltered;

	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(rpy[2], Eigen::Vector3f::UnitZ())
  		* Eigen::AngleAxisf(rpy[1], Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(rpy[0], Eigen::Vector3f::UnitX());
	// Acc from body frame to world frame
	accel = m * accel;

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

	if (initCounter < initTime) {
		init_acc += accel;
		init_gyr += gyro;
		initCounter++;
	}
	else if (initCounter == initTime) {
		init_acc /= initTime;
		gravity = init_acc.norm();
		setGravity(gravity);
		ROS_INFO("IMU Calibration Done! Gravity: %f", gravity);
		initCounter++;

		setInit(msg->header.stamp);
	}
	else {
		processUpdate(accel, gyro, msg->header.stamp);
		measureUpdate(range_al);
	}
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
		motor1_des = rotor_ready_count / 150 * pm.motor_const1;
		motor2_des = 0;
		return false;
	}
	else if (rotor_ready_count < 300) {
		motor1_des = pm.motor_const1;
		motor2_des = (rotor_ready_count - 150) / 150 * pm.motor_const2;
		return false;
	}
	return true;	
}

void CoaxVisionControl::stabilizationControl(void) {
	double Dyaw,Dyaw_rate,yaw_control;
	double Droll,Droll_rate,roll_control;
	double Dpitch,Dpitch_rate,pitch_control;
	double altitude_control,Daltitude, Daltitude_rate;
	static double Daltitude_Int = 0;

	Eigen::Vector2f state = getState();

	altitude_des = rpyt_rc[3];
	Daltitude =  altitude_des - state(0);
	Daltitude_rate = -state(1);
	Daltitude_Int += Daltitude;
	altitude_control = pm.kp_altitude * Daltitude + pm.kd_altitude * Daltitude_rate + pm.ki_altitude * Daltitude_Int;

//	ROS_INFO("range %f Daltitude %f",range_al,Daltitude);
	// yaw error and ctrl
	yaw_des += pm.yaw_coef1*(rpyt_rc[2]+rpyt_rc_trim[2]);
	ROS_INFO("desired yaw %f", yaw_des);
	Dyaw = rpy[2] - yaw_des;
	Dyaw_rate = gyro[2] - yaw_rate_des; 
	yaw_control = pm.kp_yaw * Dyaw + pm.kd_yaw * Dyaw_rate; 
//	ROS_INFO("rc yaw %f", pm.yaw_coef1*(rc_y+rc_trim_y));

	// roll error and ctrl
	Droll = rpy[0] - roll_des;
	Droll_rate = gyro[0] - roll_rate_des;
	roll_control = pm.kp_roll * Droll + pm.kd_roll * Droll_rate;

	// pitch error and ctrl
	Dpitch = rpy[1] - pitch_des;
	Dpitch_rate = gyro[1] - pitch_rate_des;
	pitch_control = pm.kp_pitch * Dpitch + pm.kd_pitch * Dpitch_rate;

	// desired motor & servo output
	motor1_des = pm.motor_const1 - yaw_control + altitude_control;
	motor2_des = pm.motor_const2 + yaw_control + altitude_control;
	servo1_des = pm.servo1_const + pm.r_rc_coef * (rpyt_rc[0]+rpyt_rc_trim[0]) + roll_control;
	servo2_des = pm.servo2_const - pm.p_rc_coef * (rpyt_rc[1]+rpyt_rc_trim[1]) + pitch_control;
}

void CoaxVisionControl::controlPublisher(size_t rate) {
	ros::Rate loop_rate(rate);

	while(ros::ok()) {

		if (rotorReady()) {
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
	CoaxVisionControl control(nh);

	// Load Params
	control.loadParams(nh);

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




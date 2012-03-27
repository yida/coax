#ifndef __COAX_VISION_CONTROL__
#define __COAX_VISION_CONTROL__

#define CONTROL_LANDED 0 // State when helicopter has landed successfully                           
#define CONTROL_START 1 // Start / Takeoff                                                          
#define CONTROL_TRIM 2 // Trim Servos                                                               
#define CONTROL_HOVER 3 // Hover                                                                    
#define CONTROL_GOTOPOS 4 // Go to Position                                                         
#define CONTROL_TRAJECTORY 5 // Follow Trajectory                                                   
#define CONTROL_LANDING 6 // Landing maneuver

#include <std_msgs/String.h>

#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxRawControl.h>
#include <coax_msgs/CoaxControl.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxSetTimeout.h>

#include <coax_vision/SetNavMode.h>
#include <coax_vision/SetControlMode.h>

#include <VisionFeedback.h>
#include <CoaxFilter.h>

struct param {
	double motor_const1;
	double motor_const2;
	double servo1_const;
	double servo2_const;
	double yaw_coef1;
	double yaw_coef2;
	double thr_coef1;
	double thr_coef2;
	double r_rc_coef;
	double p_rc_coef;
	double kp_yaw;
	double kd_yaw;
	double kp_roll;
	double kd_roll;
	double kp_pitch;
	double kd_pitch;
	double kp_altitude;
	double kp_imgyaw;
	double kp_imgroll;
	double range_base;
};


class CoaxVisionControl : public VisionFeedback, public KF 
{

public:
	CoaxVisionControl(ros::NodeHandle&);
	~CoaxVisionControl();

	void loadParams(ros::NodeHandle & n);
	bool reachNavState(int des_state, float timeout);
	bool configureComm(int frequency, int contents);
	bool configureControl(size_t rollMode, size_t pitchMode, size_t yawMode, size_t altitudeMode);
	bool setTimeout(size_t control_timeout_ms, size_t watchdog_timeout_ms);

	bool setNavMode(coax_vision::SetNavMode::Request &req, coax_vision::SetNavMode::Response &out);
	bool setControlMode(coax_vision::SetControlMode::Request &req, coax_vision::SetControlMode::Response &out);

	void coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message);
	void stabilizationControl(void);
	void imuAnalysis(void);
	void visionControl(void);
	bool rotorReady(void);
	void controlPublisher(size_t rate);
	
	bool setRawControl(double motor_up,double motor_lo, double servo_ro,double servo_pi);



private:
	ros::ServiceClient reach_nav_state;
	ros::ServiceClient configure_comm;
	ros::ServiceClient configure_control;
	ros::ServiceClient set_timeout;
	
	ros::Subscriber coax_state_sub;

	ros::Publisher raw_control_pub;
	ros::Publisher vision_control_pub;

	std::vector<ros::ServiceServer> set_nav_mode;
	std::vector<ros::ServiceServer> set_control_mode;

	bool LOW_POWER_DETECTED;

	int CONTROL_MODE;
	bool FIRST_START;
	bool FIRST_STATE;
	bool FIRST_LANDING;
	bool FIRST_HOVER;
	bool INIT_DESIRE;
	bool INIT_IMU;

	bool coax_nav_mode;
	bool coax_control_mode;
	int coax_state_age;
	int raw_control_age;
	int init_count;
	int init_imu_count;
	int rotor_ready_count;
	double last_state_time;
	
	double battery_voltage;
	struct param pm;
	
	double init_imu_x;
	double init_imu_y;
	double init_imu_z;

	double imu_y; // imu yaw
	double imu_r; // imu roll 
	double imu_p; // imu pitch
	double imu_al; // altitude by integrate imu
	double range_al; // range altitude

	double rc_th; // rc throttle
	double rc_y;  // rc yaw
	double rc_r;  // rc roll
	double rc_p;  // rc pitch
	double rc_trim_th; // rc throttle trim
	double rc_trim_y;  // rc yaw trim
	double rc_trim_r;  // rc roll trim
	double rc_trim_p;  // rc pitch trim
	
	double img_th;  // image throttle
	double img_y; 	// image yaw
	double img_r; 	// image roll
	double img_p; 	// image pitch

	double gyro_ch1;
	double gyro_ch2;
	double gyro_ch3;
	double accel_x;
	double accel_y;
	double accel_z; 
	double gyro_ch1_init;
	double gyro_ch2_init;
	double gyro_ch3_init;
	double accel_x_init;
	double accel_y_init;
	double accel_z_init;
	double gravity;

	double pos_z;
	double vel_z;

	double motor_up;
	double motor_lo;
	double servo_roll;
	double servo_pitch;
	double roll_trim;
	double pitch_trim;


	double motor1_des;
	double motor2_des;
	double servo1_des;
	double servo2_des;

	double yaw_des;
	double yaw_rate_des;
	double roll_des;
	double roll_rate_des;
	double pitch_des;
	double pitch_rate_des;
	double altitude_des;

	double z;
	double z_v;


};


#endif

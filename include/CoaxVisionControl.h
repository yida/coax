#ifndef __COAX_VISION_CONTROL__
#define __COAX_VISION_CONTROL__

#define CONTROL_LANDED 0 // State when helicopter has landed successfully                           
#define CONTROL_START 1 // Start / Takeoff                                                          
#define CONTROL_TRIM 2 // Trim Servos                                                               
#define CONTROL_HOVER 3 // Hover                                                                    
#define CONTROL_GOTOPOS 4 // Go to Position                                                         
#define CONTROL_TRAJECTORY 5 // Follow Trajectory                                                   
#define CONTROL_LANDING 6 // Landing maneuver

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxRawControl.h>
#include <coax_msgs/CoaxControl.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxSetTimeout.h>

#include <coax_vision/SetNavMode.h>
#include <coax_vision/SetControlMode.h>
//#include <coax_vision/CoaxFMdes.h>

using namespace std;

struct SymAxis 
{
	unsigned short axis;
	double value;
};

class CompareSymAxis 
{
public:
	bool operator() (SymAxis& A1, SymAxis& A2); 
};

class ImageProc {
	friend class CoaxVisionControl;


public:
  ImageProc(ros::NodeHandle&);
  ~ImageProc();

private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber Sub_Image;
  image_transport::Publisher Pub_Image;
	// For debug message
	ros::Publisher Debug_Msgs;

  void proc(const sensor_msgs::ImageConstPtr& msg);

	size_t width;
	size_t height;
	size_t symPos;
};



class CoaxVisionControl 
{
	friend class ImageProc;

public:
	CoaxVisionControl(ros::NodeHandle &, ImageProc &);
	~CoaxVisionControl();

	bool reachNavState(int des_state, float timeout);
	bool configureComm(int frequency, int contents);
	bool configureControl(size_t rollMode, size_t pitchMode, size_t yawMode, size_t altitudeMode);
	bool setTimeout(size_t control_timeout_ms, size_t watchdog_timeout_ms);

	bool setNavMode(coax_vision::SetNavMode::Request &req, coax_vision::SetNavMode::Response &out);
	bool setControlMode(coax_vision::SetControlMode::Request &req, coax_vision::SetControlMode::Response &out);

	void coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message);

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

	vector<ros::ServiceServer> set_nav_mode;
	vector<ros::ServiceServer> set_control_mode;

	bool LOW_POWER_DETECTED;

	int CONTROL_MODE;
	bool FIRST_START;
	bool FIRST_LANDING;
	bool FIRST_HOVER;

	bool coax_nav_mode;
	bool coax_control_mode;
	int coax_state_age;
	int raw_control_age;
	
	double battery_voltage;
	
	double imu_y; // imu yaw
	double imu_r; // imu roll 
	double imu_p; // imu pitch
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

	double motor_up;
	double motor_lo;
	double servo_roll;
	double servo_pitch;
	double roll_trim;
	double pitch_trim;

	double motor_coef1;
	double motor_coef2;

	double yaw_coef1;
	double yaw_coef2;

};


#endif

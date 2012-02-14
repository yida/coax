#ifndef __COAX_VISION_CONTROL__
#define __COAX_VISION_CONTROL__

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxRawControl.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxSetTimeout.h>

//#include <coax_vision/SetControlMode.h>
//#include <coax_vision/CoaxFMdes.h>

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
  void proc(const sensor_msgs::ImageConstPtr& msg);
  image_transport::ImageTransport it_;
  image_transport::Subscriber Sub_Image;
  image_transport::Publisher Pub_Image;
	// For debug message
	ros::Publisher Debug_Msgs;
};



class CoaxVisionControl 
{
	friend class ImageProc;

public:
	CoaxVisionControl(ros::NodeHandle &);
	~CoaxVisionControl();

private:
	ros::ServiceClient reach_nav_state;
	ros::ServiceClient configure_comm;
	ros::ServiceClient set_timeout;
	
	ros::Subscriber coax_fmdes_sub;
	ros::Subscriber coax_state_sub;

};

using namespace std;

#endif

#ifndef __COAX_VISION_CONTROL__
#define __COAX_VISION_CONTROL__

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
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber Sub_Image;
  image_transport::Publisher Pub_Image;
	// For debug message
	ros::Publisher Debug_Msgs;

public:
  ImageProc();
  ~ImageProc();

private:
  void proc(const sensor_msgs::ImageConstPtr& msg);
};



class CoaxVisionControl 
{
	friend class ImageProc;

public:
	CoaxVisionControl();
	~CoaxVisionControl();

private:

};

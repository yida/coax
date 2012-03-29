#ifndef __VISIONFEEDBACK__
#define __VISIONFEEDBACK__

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <ros/ros.h>

#include <coax_vision/ImageDebug.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>

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

class VisionFeedback {
	friend class CoaxVisionControl;


public:
  VisionFeedback(ros::NodeHandle&);
  ~VisionFeedback();

private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber Sub_Image;
  image_transport::Publisher Pub_Image;
	// For debug message
	ros::Publisher Debug_Msgs;

  void proc(const sensor_msgs::ImageConstPtr& msg);
	// Flip Searching Symmetric Axis
	std::deque<SymAxis> SortedAxis;
	std::deque<SymAxis> PeakAxis;
	std::vector<double> LastOpicFlow;
	bool FIRST_FRAME;
	size_t width;
	size_t height;
	size_t symPos;
	double shift;
};

#endif

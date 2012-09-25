#ifndef __VISIONFEEDBACK__
#define __VISIONFEEDBACK__

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <ros/ros.h>
#include <Eigen/Dense>

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

  void proc(const sensor_msgs::ImageConstPtr& msg);
	void resize(const sensor_msgs::ImageConstPtr& msg);
	void PublishImage(const Eigen::MatrixXd& img, image_transport::Publisher& Pub);

private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber Sub_Image;
  image_transport::Publisher Pub_Image;
	// For debug message
	ros::Publisher Debug_Msgs;

	// Flip Searching Symmetric Axis
	std::deque<SymAxis> SortedAxis;
	std::deque<SymAxis> PeakAxis;
	std::vector<double> LastOpicFlow;
	bool FIRST_FRAME;
	int width;
	int height;
	double resize_ratio;
	size_t symPos;
	double shift;

	Eigen::MatrixXd image;
};

#endif

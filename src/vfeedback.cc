#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <sstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <coax_msgs/CoaxState.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
namespace enc = sensor_msgs::image_encodings;

struct SymAxis {
	unsigned short axis;
	double value;
};

class CompareSymAxis {
	public:
	bool operator() (SymAxis& A1, SymAxis& A2) {
		if (A1.value > A2.value)
			return true;
		else
			return false;
	}
};

static const char WINDOW[] = "Image window";

class ImageProc {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber Sub_Image;
  image_transport::Publisher Pub_Image;
	// For debug message
	ros::Publisher Debug_Msgs;

public:
  ImageProc(): it_(nh_) {
		Sub_Image = it_.subscribe("/camera/image_raw", 1 ,&ImageProc::proc, this);
    Pub_Image = it_.advertise("image_out", 1);
		Debug_Msgs = nh_.advertise<std_msgs::String>("debug",100);

		cv::namedWindow(WINDOW);
  }

  ~ImageProc() {
		cv::destroyWindow(WINDOW);
  }

  void proc(const sensor_msgs::ImageConstPtr& msg) {
		sensor_msgs::Image frame;
		frame = *msg;
		// Resize
		unsigned int resize = 10; 
		frame.height = msg->height / resize;
    frame.width = msg->width / resize;
    frame.step = msg->width / resize;
		frame.data.resize(frame.height * frame.width);
    unsigned int im_idx = 0;
    for (unsigned int i = 0; i < msg->height; i += resize)
		  for (unsigned int j = 0; j < msg->width; j += resize) {
				frame.data[im_idx] = msg->data[i*msg->step+j];
				im_idx ++;
			}

		// Inegral Image
		vector <uint32_t> Integral_Image;
		Integral_Image.resize(frame.height * frame.width);
		for (unsigned int i = 0; i < frame.width; i++)
	  	for (unsigned int j = 0; j < frame.height; j++) {
				unsigned int Idx = j * frame.width + i;
				unsigned int Up_Idx = (j-1) * frame.width + i;
				unsigned int Left_Idx = j * frame.width + i - 1;
				unsigned int UpLeft_Idx = (j-1) * frame.width + i - 1;
				uint8_t Up = (j == 0)? 0 : frame.data[Up_Idx];
				uint8_t Left = (i == 0)? 0 : frame.data[Left_Idx];
				uint8_t UpLeft = ((i == 0) || (j == 0))? 0 : frame.data[UpLeft_Idx];
				Integral_Image[Idx] = frame.data[Idx] + Up + Left - UpLeft;  
	  }

		// Flip Searching Symmetric Axis
		priority_queue<SymAxis, vector<SymAxis>, CompareSymAxis> Axis;
		unsigned int shift = (frame.height - 1) * frame.width;
		double Left_Sum = 0;
		double Right_Sum = 0;
		for (unsigned short i = 1; i < (frame.width - 1); i++) {
			if ( i < frame.width/2 ) {
				Left_Sum = Integral_Image[shift+(i-1)];
				Right_Sum = Integral_Image[shift+2*i] - Integral_Image[shift+i];
			}
			else {
				Left_Sum = Integral_Image[shift+i-1] - Integral_Image[shift+2*i-frame.width];
				Right_Sum = Integral_Image[shift+frame.width-1] - Integral_Image[shift+i];
			}
			SymAxis temp_axis = {i,abs(Left_Sum-Right_Sum)};
			Axis.push(temp_axis);
		}
		SymAxis Best = Axis.top();
		// cv windows to show symmetric line
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(frame,enc::MONO8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::circle(cv_ptr->image, cv::Point(Best.axis,frame.height/2), 5, CV_RGB(255,0,0));
		// Display labeled image
//		cv::imshow(WINDOW, cv_ptr->image);
//		cv::waitKey(3);
		Pub_Image.publish(cv_ptr->toImageMsg());

//		std_msgs::String pub_msgs;
//		std::stringstream ss;
//		while (!Axis.empty()) {
//			SymAxis temp_axis = Axis.top();
//			ss << temp_axis.axis << ":" << temp_axis.value << " ";
//			Axis.pop();
//		}
//	  ss << endl;
//		pub_msgs.data = ss.str();
//		ROS_INFO("%s",pub_msgs.data.c_str());
//		cout << ss << endl;	
//		Debug_Msgs.publish(pub_msgs);
//		Pub_Image.publish(frame);
	}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vfeedback");
  ImageProc img;  

  ros::spin();
  return 0;
}




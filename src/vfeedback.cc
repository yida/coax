#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <coax_msgs/CoaxState.h>

using namespace std;

class ImageProc {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber Sub_Image;
  image_transport::Publisher Pub_Image;

public:
  ImageProc(): it_(nh_) {
	Sub_Image = it_.subscribe("/camera/image_raw", 1 ,&ImageProc::proc, this);
    Pub_Image = it_.advertise("image_out", 1);
  }

  ~ImageProc() {
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
	vector <uint8_t> Integral_Image;
	Integral_Image.resize(frame.height * frame.width);
	for (unsigned int i = 0; i < frame.width; i ++)
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

	Pub_Image.publish(frame);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vfeedback");
  ImageProc img;  

  ros::spin();
  return 0;
}

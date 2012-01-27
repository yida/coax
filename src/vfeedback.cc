#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <coax_msgs/CoaxState.h>

using namespace std;

ros::NodeHandle *node;
sensor_msgs::Image input;
sensor_msgs::Image output;
coax_msgs::CoaxState coax;
//ros::Publisher PubImage;
image_transport::Publisher PubImage;

bool new_image = false;
bool new_state = false;

void image_callback(const sensor_msgs::ImageConstPtr& image) {
  input = *image;
  new_image = true;
  return;
}

void coax_callback(const coax_msgs::CoaxStateConstPtr& state) {
  coax = *state;
  new_state = true;
  return;
}

bool spin() {
  while (node->ok()) {
    ros::spinOnce();
    if (new_image) {
      output = input;
      output.height = 24;
      output.width = 32;
      output.step = 32;
	  int im_idx = 0;
	  for (unsigned int i = 0; i < input.height; i+=20)
		for (unsigned int j = 0; j < input.width; j+=20) {
//		  cout << im_idx << ' ';
		  output.data[im_idx] = output.data[i*input.step+j];
		  im_idx ++;
//	  	  cout << (unsigned short)output.data[i*input.step+j] << ' ';
	  }
//	  cout << "new image" << endl;
      PubImage.publish(output);
      new_image = false;
    }
    if (new_state) {
      cout << "State Time Stamp " << coax.header.stamp << '.';
      //cout << coax.header.stamp.nsecs << ' ';
      cout << "Yaw " << coax.yaw << endl;
      new_state = false;
    }
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vfeedback");
  ros::NodeHandle n;
  node = &n;
  
  image_transport::ImageTransport it(*node);
  image_transport::Subscriber SubImage = it.subscribe("/camera/image_raw",1, image_callback);
  PubImage = it.advertise("image_resize",1);
  
  spin();
  return 0;
}

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <coax_msgs/CoaxState.h>

using namespace std;

ros::NodeHandle *node;
sensor_msgs::Image frame;
coax_msgs::CoaxState coax;

bool new_image = false;
bool new_state = false;

void image_callback(const sensor_msgs::ImageConstPtr& image) {
  frame = *image;
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
      cout << "Image Time Stamp " << frame.header.stamp << ' ';
      cout << frame.height << ' ' << frame.width << endl;  
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
  
  ros::Subscriber SubImage = n.subscribe("/camera/image_raw", 50, image_callback);
  ros::Subscriber SubState = n.subscribe("/coax_server/state",1000, coax_callback);

  spin();
  return 0;
}

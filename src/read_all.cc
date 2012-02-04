#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <coax_msgs/CoaxState.h>
#include <image_transport/image_transport.h>


using namespace std;
ros::NodeHandle *node;
sensor_msgs::Image frame;
coax_msgs::CoaxState coax;

bool new_image = false;

void image_callback(const sensor_msgs::Image::ConstPtr& img) {
	frame = *img;
	new_image = true;
	return;
}

void state_callback(const coax_msgs::CoaxState::ConstPtr& state) {
	coax = *state;
	return;
}

bool spin() {
//	ros::Rate r(15.0);
	while (node->ok()) {
//		ROS_INFO("update");
		ros::spinOnce();
		if (new_image) {
			cout << "Image Time Stamp: " << setprecision(15) << frame.header.stamp.toSec() << " ";
			cout << "State Time Stamp: " << coax.header.stamp.toSec() << endl;
			new_image = false;
		}
//		r.sleep();
	}
	return true;
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "read_all");
	ros::NodeHandle n;
	node = &n;

	image_transport::ImageTransport it_(*node);
	image_transport::Subscriber Sub_Image = it_.subscribe("/camera/image_raw",1,image_callback);
	ros::Subscriber sub = n.subscribe("/coax_server/state", 200, state_callback);

	spin();
	return 0;


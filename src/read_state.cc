#include <iostream>
#include <iomanip>
#include <fstream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <ros/ros.h>
#include <coax_msgs/CoaxState.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
namespace fs = boost::filesystem;
ros::NodeHandle *node;
ofstream* output;
coax_msgs::CoaxState coax;
sensor_msgs::Image frame;

bool new_image = false;
int cnt = 0;

void imageCallback(const sensor_msgs::Image::ConstPtr& img) {
			frame = *img;
			cout << cnt << " " << setprecision(15) <<frame.header.stamp.toSec() << " " << coax.header.stamp.toSec() << endl;
			*output << cnt << " ";
			*output << setprecision(15) << frame.header.stamp.toSec() << " ";
			*output << setprecision(15) << coax.header.stamp.toSec() << " ";
			*output << (unsigned short)coax.mode.navigation << " ";
			*output << (unsigned short)coax.mode.communication << " ";
			*output << (unsigned short)coax.mode.oavoid << " ";
			*output << (unsigned short)coax.mode.rollAxis << " ";
			*output << (unsigned short)coax.mode.pitchAxis << " ";
			*output << (unsigned short)coax.mode.yawAxis << " ";
			*output << (unsigned short)coax.mode.altAxis << " ";
			
			*output << coax.roll << " ";
			*output << coax.pitch << " ";
			*output << coax.yaw << " ";
			
			*output << coax.gyro[0] << " " << coax.gyro[1] << " " << coax.gyro[2] << " ";
			*output << coax.accel[0] << " " << coax.accel[1] << " " << coax.accel[2] << " ";
		
		  *output << coax.zrange << " ";
			*output << coax.zfiltered << " ";
			*output << coax.battery << " ";
		
			*output << coax.rcChannel[0] << " ";
			*output << coax.rcChannel[1] << " ";
			*output << coax.rcChannel[2] << " ";
			*output << coax.rcChannel[3] << " ";
			*output << coax.rcChannel[4] << " ";
			*output << coax.rcChannel[5] << " ";
			*output << coax.rcChannel[6] << " ";
			*output << coax.rcChannel[7] << " ";	
			*output << endl;
			cnt ++;
}

void stateCallback(const coax_msgs::CoaxState::ConstPtr & message) {
	coax = *message;
	return;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "read_state");
	ros::NodeHandle nh_;
	node = &nh_;
	image_transport::ImageTransport it_(*node);
	image_transport::Subscriber Sub_Image = it_.subscribe("/camera/image_raw",1,imageCallback);
	ros::Subscriber sub = nh_.subscribe("/coax_server/state",200,stateCallback);

	string directory;
	nh_.param("directory", directory, string("data"));

	string filename;
	nh_.param("filename", filename, string("CoaxState"));

	fs::path path(directory, fs::native);

  if (!fs::exists(path)) {
		if (!path.is_complete())
			 path = fs::current_path() / path;
		
		if (!fs::create_directory(path)) {   
    	ROS_ERROR("%s: Failed to locate or create data directory:\n%s",
      					ros::this_node::getName().c_str(),
          			path.string().c_str());
	    return -1;
		}   
	  else
      ROS_INFO("%s: Created data directory:\n%s",
         			ros::this_node::getName().c_str(),
            	path.string().c_str());
	}   

	fs::path file = path / filename;
  string file_string(file.string());
  string file_string_ext(file_string);

  file = fs::path(file_string_ext);

	ROS_INFO("Writing file: %s", file.string().c_str());

	output = new ofstream(file.string().c_str(), ios_base::out);
	output->precision(18);
	ros::spin();
	output->close();

	return 0;	
}

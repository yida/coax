#include <iostream>
#include <iomanip>
#include <fstream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <ros/ros.h>
#include <coax_msgs/CoaxState.h>

using namespace std;
namespace fs = boost::filesystem;
ofstream* output;
coax_msgs::CoaxState msg;
void stateCallback(const coax_msgs::CoaxState::ConstPtr & message) {
	msg = *message;
	cout << setprecision(15) << msg.header.stamp.toSec() << endl;

	*output << setprecision(15) << msg.header.stamp.toSec() << " ";
	*output << (unsigned short)msg.mode.navigation << " ";
	*output << (unsigned short)msg.mode.communication << " ";
	*output << (unsigned short)msg.mode.oavoid << " ";
	*output << (unsigned short)msg.mode.rollAxis << " ";
	*output << (unsigned short)msg.mode.pitchAxis << " ";
	*output << (unsigned short)msg.mode.yawAxis << " ";
	*output << (unsigned short)msg.mode.altAxis << " ";
	
	*output << msg.roll << " ";
	*output << msg.pitch << " ";
	*output << msg.yaw << " ";
	
	*output << msg.gyro[0] << " " << msg.gyro[1] << " " << msg.gyro[2] << " ";
	*output << msg.accel[0] << " " << msg.accel[1] << " " << msg.accel[2] << " ";

  *output << msg.zrange << " ";
	*output << msg.zfiltered << " ";
	*output << msg.battery << " ";

	*output << msg.rcChannel[0] << " ";
	*output << msg.rcChannel[1] << " ";
	*output << msg.rcChannel[2] << " ";
	*output << msg.rcChannel[3] << " ";
	*output << msg.rcChannel[4] << " ";
	*output << msg.rcChannel[5] << " ";
	*output << msg.rcChannel[6] << " ";
	*output << msg.rcChannel[7] << " ";
	
	*output << endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "read_state");
	ros::NodeHandle nh_;
	ros::Subscriber sub = nh_.subscribe("state",200,stateCallback);

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

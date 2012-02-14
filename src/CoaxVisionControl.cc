#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <ros/ros.h>
#include <coax_msgs/CoaxState.h>

#include <com/sbapi.h>
#include <CoaxVisionControl.h>

CoaxVisionControl::CoaxVisionControl(ros::NodeHandle &node) 
{
	
}

CoaxVisionControl::~CoaxVisionControl()
{
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "CoaxVisionControl");
	ros::NodeHandle nh("/coax_vision_control");
  ImageProc vision(nh);  
	CoaxVisionControl control(nh);

	// make sure coax_server has enough time to start
	ros::Duration(1.5).sleep(); 
  ros::spin();
  return 0;
}




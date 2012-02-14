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

int main(int argc, char **argv) {
  ros::init(argc, argv, "CoaxVisionControl");
	ros::NodeHandle nh("/coax_vision_control");
  ImageProc vision(nh);  
	CoaxVisionControl control(nh);

  ros::spin();
  return 0;
}




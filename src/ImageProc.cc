#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <ros/ros.h>

#include <CoaxVisionControl.h>
#include <coax_vision/ImageDebug.h>

namespace enc = sensor_msgs::image_encodings;

bool CompareSymAxis::operator() (SymAxis& A1, SymAxis& A2) 
{
	if (A1.value < A2.value)
		return true;
	else
		return false;
};

ImageProc::ImageProc(ros::NodeHandle& nh_)
:it_(nh_) 
,Sub_Image(it_.subscribe("image_in", 1 ,&ImageProc::proc, this))
,Pub_Image(it_.advertise("image_proc", 1))
,Debug_Msgs(nh_.advertise<coax_vision::ImageDebug>("image_debug",100))
,width(0)
,height(0)
,symPos(0)
{
}

ImageProc::~ImageProc() {
}

void ImageProc::proc(const sensor_msgs::ImageConstPtr& msg) 
{
//	ROS_INFO("frame type %d",msg->step);
	sensor_msgs::Image frame;
	frame = *msg;
	// Resize
	size_t resize = 10; 
	frame.height = msg->height / resize;
  frame.width = msg->width / resize;
  frame.step = msg->width / resize;
	frame.data.resize(frame.height * frame.width);
  size_t im_idx = 0;
  for (size_t i = 0; i < msg->height; i += resize)
	  for (size_t j = 0; j < msg->width; j += resize) {
			frame.data[im_idx] = msg->data[i*msg->step+j];
			im_idx ++;
		}

	// Inegral Image
	vector <uint32_t> Integral_Image;
	size_t Idx = 0;          
	size_t Up_Idx = 0;       
	size_t Left_Idx = 0;     
	size_t UpLeft_Idx = 0;   
	uint8_t Up = 0;                
	uint8_t Left = 0;              
	uint8_t UpLeft = 0;            
	Integral_Image.resize(frame.height * frame.width);
	for (size_t i = 0; i < frame.width; i++)
  	for (size_t j = 0; j < frame.height; j++) {
			Idx = j * frame.width + i;
			Up_Idx = (j-1) * frame.width + i;
			Left_Idx = j * frame.width + i - 1;
			UpLeft_Idx = (j-1) * frame.width + i - 1;
			Up = (j == 0)? 0 : frame.data[Up_Idx];
			Left = (i == 0)? 0 : frame.data[Left_Idx];
			UpLeft = ((i == 0) || (j == 0))? 0 : frame.data[UpLeft_Idx];
			Integral_Image[Idx] = frame.data[Idx] + Up + Left - UpLeft;  
  }

	// Flip Searching Symmetric Axis
	priority_queue<SymAxis, vector<SymAxis>, CompareSymAxis> Axis;
	
	// Integral Image Based, abs(sum(left)-sum(right))
//	size_t shift = (frame.height - 1) * frame.width;
//	double Left_Sum = 0;
//	double Right_Sum = 0;
//	for (unsigned short i = 1; i < (frame.width - 1); i++) {
//		if ( i < frame.width/2 ) {
//			Left_Sum = Integral_Image[shift+(i-1)];
//			Right_Sum = Integral_Image[shift+2*i] - Integral_Image[shift+i];
//		}
//		else {
//			Left_Sum = Integral_Image[shift+i-1] - Integral_Image[shift+2*i-frame.width];
//			Right_Sum = Integral_Image[shift+frame.width-1] - Integral_Image[shift+i];
//		}
//		SymAxis temp_axis = {i,abs(Left_Sum-Right_Sum)};
//		Axis.push(temp_axis);
//	}

	// Kernel based , L1
	size_t width = 0;
	double Sum_L1_Norm = 0;	
	size_t shift = 0;
	size_t Left_Cur = 0;
	size_t Right_Cur = 0; 
	double L1_Norm = 0;
	for (size_t cnt = 1; cnt < (frame.width - 1); cnt++) {
		width = min(cnt,frame.width-1-cnt);
		Sum_L1_Norm = 0;
		for (size_t row = 0; row < frame.height; row++) {
			shift = row * frame.width;
			for (size_t cur = 1; cur <= width; cur++) {
				Left_Cur = cnt - cur;
				Right_Cur = cnt + cur;
				L1_Norm = abs(frame.data[shift+Left_Cur] - frame.data[shift+Right_Cur]);
				Sum_L1_Norm += L1_Norm;
			}
		}
		SymAxis temp_axis = {cnt, Sum_L1_Norm};
		Axis.push(temp_axis);
	}
	
			
	SymAxis Best = Axis.top();
	//ROS_INFO("Axis %d",Best.axis);
	
	// Generate Debug Message
	coax_vision::ImageDebug debugMsg;
	debugMsg.header.stamp = ros::Time::now();
	debugMsg.header.seq = 0;
	debugMsg.header.frame_id = "image_debug";
	debugMsg.BestAxis = Best.axis;
	Debug_Msgs.publish(debugMsg);	
	Pub_Image.publish(frame);
}



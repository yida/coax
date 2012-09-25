#include <string.h>
#include <com/sbapi.h>
#include <com/sbchannel.h>
#include <com/sbmessage.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "coax_webots_tf_broadcaster");
  if (argc < 2) {
      printf("Usage: %s <host>\n",argv[0]);
      return -1;
  }
  SBChannel pose_channel;
  sbChannelCreateSocketUDP(&pose_channel,argv[1],5125);
  sbChannelOpen(&pose_channel);
  sbChannelSend(&pose_channel,(const unsigned char*)argv[0],strlen(argv[0]));
  printf("Opened pose channel on UDP ports 5125\n");

  ros::NodeHandle node;
  tf::TransformBroadcaster br;

  while (ros::ok()) {
      SBSerialisedMessage reply;
      SB6DofPoseMessage pose;
      ros::spinOnce();

      if (sbChannelWaitData(&pose_channel, 100)) {
          continue;
      }
      // printf("\n1");
      if (sbWaitRawMessage(&pose_channel,-1,&reply,100)) {
          continue;
      }
      // printf("2");
      if ((reply.msgid & SB_MSGID_MASK) != SB_MSGID_6DOF_POSE) {
          continue;
      }
      // printf("3");
      if (sb6DofPoseDecode(&reply,&pose)) {
          continue;
      }
      //printf("[%ld %ld %ld] [%d %d %d]\n",
      //        pose.x,pose.y,pose.z,pose.roll,pose.pitch,pose.yaw);
      // printf("4");
      // fflush(stdout);
      tf::Transform transform;
      tf::Quaternion q;
      transform.setOrigin( tf::Vector3(pose.x/1000., pose.y/1000., pose.z/1000.) );
      q.setRPY(pose.roll*M_PI/0x8000,pose.pitch*M_PI/0x8000,pose.yaw*M_PI/0x8000);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "coax"));
  }
  return 0;
};


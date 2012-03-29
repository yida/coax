#ifndef __COAXFILTER__
#define __COAXFILTER__

#include <ros/ros.h>
#include <Eigen/Dense>

class KF {
public:
	KF(ros::NodeHandle&);
	~KF();

	bool processUpdate(const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro,
											const ros::Time& time);
	bool measureUpdate(const double& range);

	void setGravity(double& g);
	void setInit(const ros::Time& time);
	Eigen::Vector2f getState(void);
	Eigen::Matrix2f getCovariance(void);
private:

	double gravity;
	Eigen::Vector2f state;
	Eigen::Matrix2f cov;
	double Time;
	double varAcc;
	double varSonar;


};

#endif

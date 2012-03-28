
#include <CoaxFilter.h>

KF::KF()
:gravity(9.8)

,Time(0)
{
	state << 0.0, 0.0;
	cov << 0.0025,0.0,0.0,0.0025;
	varAcc = 0.0079;
	varSonar = 0.0001;
}

KF::~KF() {

}

void KF::setGravity(double& g) {
	gravity = g;
}

bool KF::processUpdate(const Eigen::Vector3f& acc, const Eigen::Vector3f& gryo,
												const ros::Time& time) {
	double dt = time.now().toSec() - Time; 
	Time = time.now().toSec();
	Eigen::Matrix2f A;
	A << 1, dt, 0, 1;
	Eigen::Vector2f B(0.5*dt*dt, dt);

//	std::cout << "dt : " << dt << std::endl;
	double accZ = acc.norm() - gravity;

//	std::cerr << "accZ: " << accZ << std::endl;
	state = A * state + B * accZ;	
//	std::cout << "state: \n" << state << std::endl;
	cov = A * cov * A.transpose() + B * varAcc * B.transpose();
	//	std::cout << dt << std::endl;
//	std::cout << "cov: \n" << cov << std::endl;

	return true;
}

bool KF::measureUpdate(const double& range) {
	Eigen::Vector2f C(1,0);
	Eigen::Vector2f K(0,0);
	double diffrange = range - 0.039;
//	std::cout << "diffrange: " << diffrange << std::endl;
	K = cov * C / (C.transpose()*cov*C + varSonar);
//	std::cout << "K: \n" << K << std::endl;
	state = state + K * (diffrange - C.transpose()*state);
	cov = (Eigen::Matrix2f::Identity() - K * C.transpose()) * cov;
	
//	std::cout << "matrix\n" << K << std::endl;
	return true;
}

Eigen::Vector2f KF::getState(void) {
	return state;
}

Eigen::Matrix2f KF::getCovariance(void) {
	return cov;
}

void KF::setInit(const ros::Time& time) {	
	Time = time.now().toSec();
}

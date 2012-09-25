#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef WIN32
#define sscanf sscanf_s
#define strcpy strcpy_s
#define strncpy strncpy_s
#define usleep(x) Sleep(x/1000)
#else
#include <unistd.h>
#endif

#include <com/sbapi.h>
#include <com/sbsimple.h>

#define D2R(X) ((X)*M_PI/180.0)
#define R2D(X) ((X)*180.0/M_PI)

#define DEBUG(c) res=0;c;if (res) printf("Result of "#c": %d\n",res)
#define CRITICAL(c) res=0;c;if (res) {printf("Result of "#c": %d\n",res); return res;}

int main(int argc, const char *argv[])
{
	int res=0;
	double t0;
	// Version checking
	const SBVersionStatus * compiled = sbGetCompiledVersion(); 
	unsigned int objSizeStatus = sbValidateObjectSize(compiled);
	printf("Object size status: %04x\n",objSizeStatus);
	assert(objSizeStatus == 0);


	// Configure and initialise
	SBApiSimpleContext api;
	sbSimpleDefaultContext(&api);
	// Read command line to find the type of connection
	sbSimpleParseChannel(&api,(argc>1)?argv[1]:NULL,NULL);
	// Configure control axes
	api.altCtrlMode = SB_CTRL_POS;  // Altitude is controlled in position
	api.yawCtrlMode = SB_CTRL_MANUAL;  // Yaw is controlled on the remote
	api.rollCtrlMode = SB_CTRL_POS; // Roll and pitch stay on the 
	api.pitchCtrlMode = SB_CTRL_POS;// remote
    // api.commFreq = 30;
    // api.ctrlTimeout = 4000;
	CRITICAL(res = sbSimpleInitialise(&api));
    sbLockCommunication(&api.control);
    DEBUG(res = sbConfigureAckMode(&api.control,0));
    sbUnlockCommunication(&api.control);


	// Take off to 0.3 m
	CRITICAL(res = sbSimpleReachNavState(&api,SB_NAV_CTRLLED,30.0));
	while (!(*api.endP) && (api.state.zrange < 0.25)) {
		DEBUG(res = sbSimpleControl(&api,0.0,0.0,0.0,0.4));
		printf("State: Z %f Yaw %f\n",api.state.zrange,R2D(api.state.yaw));
		usleep(20000);
	}
	// Rotate on the spot at 18 deg/s at 0.5 m
	t0 = sbGetCurrentTime();
	while (!(*api.endP) && (sbGetCurrentTime() - t0 < 120.0)) {
        double roll = sin((sbGetCurrentTime() - t0)*10)/50;
        double pitch = cos((sbGetCurrentTime() - t0)*10)/50;
		DEBUG(res = sbSimpleControl(&api,roll,pitch,0.0,0.5));
		printf("State: Z %f Yaw %f\n",api.state.zrange,R2D(api.state.yaw));
		usleep(20000);
	}
	// Land and shut down
	DEBUG(res = sbSimpleReachNavState(&api,SB_NAV_IDLE,30.0));
	DEBUG(res = sbSimpleTerminate(&api));

	return 0;
}

		

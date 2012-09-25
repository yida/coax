#include "ros/ros.h"

#define REPEAT_STRINGS
#ifdef REPEAT_STRINGS
#include "std_msgs/String.h"
#else
#include "coax_msgs/CoaxRepeat.h"
#endif

#include <com/sbapi.h>
#include <com/sbsimple.h>

//#define DEBUG(c) ROS_INFO("Executing "#c)
//#define DEBUG(c) res=0;ROS_INFO("Executing "#c);c;ROS_INFO("Result %d",res)
#define DEBUG(c) res=0;c;if (res) ROS_INFO("Result of "#c": %d",res)
#define CRITICAL(c) res=0;c;if (res) {ROS_INFO("Result of "#c": %d",res); return res;}


class SBController
{
	protected:
		SBApiSimpleContext *simple;
		int res;
		ros::Publisher *repeat_pub;
		ros::Subscriber *repeat_sub;

	public:
		SBController(SBApiSimpleContext *s) 
			: simple(s), res(0) 
		{
			repeat_pub = NULL;
			repeat_sub = NULL;
			sbSimpleDefaultContext(simple);
		}

		~SBController() {
		}

		void setupRepeatIO(ros::Publisher *pub,ros::Subscriber *sub) {
			repeat_pub = pub;
			repeat_sub = sub;
            sbRegisterRepeatCallback(&simple->control,repeat_sbapi_callback_static,this);
		}

		int initialise(const std::string & devname) {
			res = 0;
			sbSimpleParseChannel(simple,devname.c_str(),NULL);
            simple->masterMode = 0;
			simple->initNavState = SB_NAV_STOP;

			CRITICAL(res = sbSimpleInitialise(simple));
			ROS_INFO("Channel connected, continuing");
			return res;
		}

		int terminate() {
			res = 0;
			DEBUG(res = sbSimpleTerminate(simple));
			return res;
		}

        static void repeat_sbapi_callback_static(const unsigned char*data, unsigned int len, void *userData) {
            SBController * ctrller = (SBController*)userData;
            ctrller->repeat_sbapi_callback(data,len);
        }

        void repeat_sbapi_callback(const unsigned char*data, unsigned int len) {
#ifdef REPEAT_STRINGS
            std_msgs::String msg;
#else
            coax_msgs::CoaxRepeat msg;
#endif
            unsigned int i;
            msg.data.resize(len);
            for (i=0;i<len;i++) {
                msg.data[i] = data[i];
            }
            repeat_pub->publish(msg);
        }

#ifdef REPEAT_STRINGS
		void repeat_ros_callback(const std_msgs::String::ConstPtr & message) {
            if (message->data.size() > SB_STRING_MESSAGE_LENGTH) {
                ROS_ERROR("Repeat message: ignored message because it is too long (%d bytes)",message->data.size());
                return;
            }
			sbLockCommunication(&simple->control);
			DEBUG(res = sbSendRepeatMessage(&simple->control,(const unsigned char*)(message->data.c_str()),message->data.size()));
			sbUnlockCommunication(&simple->control);
		}
#else
		void repeat_ros_callback(const coax_msgs::CoaxRepeat::ConstPtr & message) {
            if (message->data.size() > SB_STRING_MESSAGE_LENGTH) {
                ROS_ERROR("Repeat message: ignored message because it is too long (%d bytes)",message->data.size());
                return;
            }
			sbLockCommunication(&simple->control);
			DEBUG(res = sbSendRepeatMessage(&simple->control,&(message->data[0]),message->data.size()));
			sbUnlockCommunication(&simple->control);
		}
#endif

};

int main(int argc, char **argv)
{
	int res;
	const SBVersionStatus * compiled = sbGetCompiledVersion(); 
	unsigned int objSizeStatus = sbValidateObjectSize(compiled);

	ros::init(argc, argv, "coax_repeat");
	ROS_INFO("Object size status: %04x",objSizeStatus);
	assert(objSizeStatus == 0);

	SBApiSimpleContext simple;
	SBController api(&simple);


	CRITICAL(res = api.initialise((argc<2)?("localhost"):(argv[1])));

	ros::NodeHandle n("/coax_repeat");


#ifdef REPEAT_STRINGS
	ros::Subscriber repeat_sub = n.subscribe("torepeat",1,&SBController::repeat_ros_callback,&api);
	ros::Publisher repeat_pub = n.advertise<std_msgs::String>("repeated",1);
#else
	ros::Subscriber repeat_sub = n.subscribe("torepeat",1,&SBController::repeat_ros_callback,&api);
	ros::Publisher repeat_pub = n.advertise<coax_msgs::CoaxRepeat>("repeated",1);
#endif
    api.setupRepeatIO(&repeat_pub,&repeat_sub);

	ROS_INFO("Coax Server ready");
	ros::spin();

	DEBUG(res = api.terminate());
	return 0;
}


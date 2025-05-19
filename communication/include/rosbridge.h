#ifndef __ROSBRIDGE_H__
#define __ROSBRIDGE_H__

#include <ros/ros.h>

#include "exterior_common_msgs/VehicleToCan.h"
#include "exterior_common_msgs/Chassis_Fb.h"
#include "exterior_common_msgs/CarStatus.h"
// #include "sensor_msgs/Joy.h"
// #include "control/ControlMode.h"
// #include "nav_msgs/Odometry.h"
// #include "tf/transform_listener.h"
#include "exterior_common_msgs/RemoteControl.h"
#include "exterior_common_msgs/cause_vehicle_halt_fb.h"


#include <iostream>

#include "include/communication_base.h"
#include "datapool.h"

#include "fmlogger/logger.hpp"

using CommunicationMethd = control_can::CommunicationBase;
using namespace control_can;

class Rosbridge: public control_can::CommunicationBase
{
public:
	Rosbridge();
	~Rosbridge() {

	}


	int Init();
	int SpinOnce();
	int Publish();

	int Start();

private:
    std::shared_ptr<CommunicationMethd> communication_;
	DataPool *DP;
	
	int InitLog();
	std::string getFormattedTime();
	// void VehicleCtrlCallBack(const vehicleControl::A2vVehicleCtrl::ConstPtr& msg);
	void VehicleToCanCallBack(const exterior_common_msgs::VehicleToCan::ConstPtr& msg);
	void RemoteControlCallBack(const exterior_common_msgs::RemoteControl::ConstPtr& msg);
	
public:
	
	// ros::Subscriber sub_VehicleCtrl_;
	ros::Subscriber sub_vehicletocan_;
	ros::Subscriber sub_remotecontrol_;
	
	ros::Publisher pub_chassis_fb_;
	ros::Publisher pub_cause_vehicle_halt_fb_;
	ros::Publisher pub_carstatus_;
	ros::NodeHandle nh_;

};



#endif

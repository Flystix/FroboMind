/*
 * testStack.cpp
 *
 *  Created on: Feb 19, 2012
 *      Author: peter
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ros/ros.h>
#include <fmMsgs/teleAir2Ground.h>
#include <fmMsgs/airframeState.h>
#include <fmMsgs/sysState.h>
#include <fmMsgs/gps_state.h>
ros::Publisher pub;
double rate;

fmMsgs::teleAir2Ground msg;

void airframeStateCallback(const fmMsgs::airframeState::ConstPtr& data) {
	msg.airframe = *data;

}

void gpsCallback(const fmMsgs::gps_state::ConstPtr& data) {
	msg.sys.gps_fix = data->fix;
	msg.sys.gps_sats = data->sat;
}

void timerCallback(const ros::TimerEvent& e) {
	pub.publish(msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "i2cnode");
	ros::NodeHandle nh;
	ros::NodeHandle np("~");

	np.param<double>("refresh_rate", rate, 10);

	ros::Subscriber sub = nh.subscribe("/airframeState", 1, airframeStateCallback);
	ros::Subscriber gps_sub = nh.subscribe("/fmExtractors/gps_state_msg", 1, gpsCallback);
	pub = nh.advertise<fmMsgs::teleAir2Ground>("/teleData", 1, false);
	ros::Timer timer = nh.createTimer(ros::Duration(1/rate), timerCallback);

	ros::spin();
}

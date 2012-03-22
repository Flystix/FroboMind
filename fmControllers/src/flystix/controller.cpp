/*
 * controller.cpp
 *
 *  Created on: Mar 15, 2012
 *      Author: peter
 */

#include <ros/ros.h>
#include "PID.hpp"
#include <fmMsgs/airframeState.h>
#include <fmMsgs/airframeControl.h>

void stateCallback(const fmMsgs::airframeState::ConstPtr& msg);
void radioCallback(const fmMsgs::airframeControl::ConstPtr& msg);
void controlPublisher(const ros::TimerEvent& e);

ros::Publisher controlPub;
PID* rollPtr;
PID* bankPtr;
PID* pitchPtr;
PID* speedPtr;
double roll = 0;

int main(int argc, char** argv) {
	ros::init(argc, argv, "flyStixController");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	ROS_INFO("flystixController : Reading parameters...");
	double rollP, rollI, rollD;
	n.param<double> ("rollP", rollP, 1);
	n.param<double> ("rollI", rollI, 1);
	n.param<double> ("rollD", rollD, 1);
	double bankP, bankI, bankD;
	n.param<double> ("bankP", bankP, 1);
	n.param<double> ("bankI", bankI, 1);
	n.param<double> ("bankD", bankD, 1);
	double pitchP, pitchI, pitchD;
	n.param<double> ("pitchP", pitchP, 1);
	n.param<double> ("pitchI", pitchI, 1);
	n.param<double> ("pitchD", pitchD, 1);
	double speedP, speedI, speedD;
	n.param<double> ("speedP", speedP, 1);
	n.param<double> ("speedI", speedI, 1);
	n.param<double> ("speedD", speedD, 1);
	double rate;
	n.param<double> ("updateRate", rate, 50);

	ROS_INFO("flystixController : Initialising control loops...");
	rollPtr  = new PID(rollP,  rollI,  rollD,  1);
	bankPtr  = new PID(bankP,  bankI,  bankD,  1);
	pitchPtr = new PID(pitchP, pitchI, pitchD, 1);
	speedPtr = new PID(speedP, speedI, speedD, 1);

	controlPub = nh.advertise<fmMsgs::airframeControl>("/servoData", 1);

	ros::Timer pub_timer = nh.createTimer(ros::Duration(1/rate), controlPublisher);

	ROS_INFO("flystixController : Subscribing to topics...");
	ros::Subscriber stateSub = nh.subscribe("/airframeState", 1, stateCallback);
	ros::Subscriber radioSub = nh.subscribe("/radioData", 1, radioCallback);

	ROS_INFO("flystixController : Spinning...");
	ros::spin();
}

void stateCallback(const fmMsgs::airframeState::ConstPtr& msg) {
	roll = msg->pose.x;
	rollPtr->setFeedback(msg->pose.x);
	pitchPtr->setFeedback(msg->pose.y);
//	bankPtr->setFeedback(msg->)
}

void radioCallback(const fmMsgs::airframeControl::ConstPtr& msg) {
	static uint8_t mode = 0;
	/*
	 * msg->aileron_left => desired bank angle
	 * msg->throttle => desired airspeed
	 * msg->rudder => desired
	 * msg->mode => [0]automatic / [1]manual flight
	 */

	rollPtr->setSetPoint((msg->aileron_left + msg->aileron_right)/2);

}

void controlPublisher(const ros::TimerEvent& e) {
	ros::Duration dt = e.current_real - e.last_real;
	fmMsgs::airframeControl msg;
	msg.stamp = ros::Time::now();
	msg.aileron_left = -rollPtr->output();
	msg.aileron_right = rollPtr->output();
	msg.throttle = speedPtr->output();

	// Mix rudder / elevator depending on roll angle

	msg.rudder = 0;
	msg.elevator = pitchPtr->output();
	controlPub.publish(msg);
}

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
//#include <fmMsgs/simData.h>
#include <math.h>

#define R2D(x) ((180*x)/M_PI)
#define D2R(x) ((M_PI*x)/180)


void stateCallback(const fmMsgs::airframeState::ConstPtr& msg);
void radioCallback(const fmMsgs::airframeControl::ConstPtr& msg);
//void simModeCallback(const fmMsgs::simData::ConstPtr& msg);
void sendServo(const ros::TimerEvent& e);

double HeadingDesired = 0;

ros::Publisher controlPub;

PID* RollPID; // Controls roll angle via ailerons; feedback = roll angle, Setpoint = stick, or other
PID* PitchPID; // Controls pitch angle via elevator; feedback = pitch angle, Setpoint = any
PID* AltPID; // Controls Altitude via pitch(PID)Angle; feedback = altitude, setpoint = desired altitude
PID* TurnPID; // Controls Yacc -> 0 via rudder; feeback = Yacc (or incline), setpoint = 0;
PID* YawPID; // Controls yaw via roll (PID); feedback = yaw; setpoint = any;

fmMsgs::airframeControl radio;
fmMsgs::airframeState state;

int main(int argc, char** argv) {
	ros::init(argc, argv, "flyStixController");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	ROS_INFO("flystixController : Reading parameters...");
	double rollP, rollI, rollD;
	int rollReverse;
	n.param<double>("rollP", rollP, 1);
	n.param<double>("rollI", rollI, 1);
	n.param<double>("rollD", rollD, 1);
	n.param<int>("rollReverse", rollReverse, 0);

	double yawP, yawI, yawD;
	int yawReverse;
	n.param<double>("yawP", yawP, 1);
	n.param<double>("yawI", yawI, 1);
	n.param<double>("yawD", yawD, 1);
	n.param<double>("yawDesired", HeadingDesired, 1);
	n.param<int>("yawReverse", yawReverse, 0);

	double pitchP, pitchI, pitchD;
	int pitchReverse;
	n.param<double>("pitchP", pitchP, 1);
	n.param<double>("pitchI", pitchI, 1);
	n.param<double>("pitchD", pitchD, 1);
	n.param<int>("pitchReverse", pitchReverse, 0);

	double speedP, speedI, speedD;
	int speedReverse;
	n.param<double>("speedP", speedP, 1);
	n.param<double>("speedI", speedI, 1);
	n.param<double>("speedD", speedD, 1);
	n.param<int>("speedReverse", speedReverse, 0);

	double altP, altI, altD;
	int altitudeReverse;
	n.param<double>("altitudeP", altP, 1);
	n.param<double>("altitudeI", altI, 1);
	n.param<double>("altitudeD", altD, 1);
	n.param<int>("altitudeReverse", altitudeReverse, 0);

	double turnP, turnI, turnD;
	int turnReverse;
	n.param<double>("turnP", turnP, 1);
	n.param<double>("turnI", turnI, 1);
	n.param<double>("turnD", turnD, 1);
	n.param<int>("turnReverse", turnReverse, 0);

	double rate;
	n.param<double>("updateRate", rate, 50);

	ROS_INFO("flystixController : Initialising control loops...");
	//Roll
	RollPID = new PID(rollP, rollI, rollD, rollReverse ? REVERSE : DIRECT);
	RollPID->SetOutputLimits(-1, 1);
	RollPID->createComputeTimer(&nh, rate); //Start the callback loop, passive until Mode=AUTOMATIC
	RollPID->SetMode(AUTOMATIC); //Start the PID
	//Pitch
	PitchPID = new PID(pitchP, pitchI, pitchD, pitchReverse ? REVERSE : DIRECT);
	PitchPID->SetOutputLimits(-1, 1);
	PitchPID->createComputeTimer(&nh, rate); //Start the callback loop, passive until Mode=AUTOMATIC
	PitchPID->SetMode(AUTOMATIC); //Start the PID
	//altitude
	AltPID = new PID(altP, altI, altD, altitudeReverse ? REVERSE : DIRECT);
	AltPID->SetOutputLimits(D2R(-30), D2R(30));
	AltPID->createComputeTimer(&nh, rate); //Start the callback loop, passive until Mode=AUTOMATIC
	AltPID->SetMode(AUTOMATIC); //Start the PID

	//Turn PID
	TurnPID = new PID(turnP, turnI, turnD, turnReverse ? REVERSE : DIRECT);
	TurnPID->SetOutputLimits(-1, 1);
	TurnPID->createComputeTimer(&nh, rate); //Start the callback loop, passive until Mode=AUTOMATIC
	TurnPID->SetMode(AUTOMATIC); //Start the PID

	//Yaw PID
	YawPID = new PID(yawP, yawI, yawD, yawReverse ? REVERSE : DIRECT);
	YawPID->SetOutputLimits(D2R(-60), D2R(60));
	YawPID->createComputeTimer(&nh, rate); //Start the callback loop, passive until Mode=AUTOMATIC
	YawPID->SetMode(AUTOMATIC); //Start the PID
	YawPID->setSetpoint(0); //Set to 0, as feedback i updated with relative error (SP-FB)%PI.

	controlPub = nh.advertise<fmMsgs::airframeControl>("/servoData", 1);

	ros::Timer pubTimer = nh.createTimer(ros::Duration(1 / rate), sendServo);

	ROS_INFO("flystixController : Subscribing to topics...");
	ros::Subscriber stateSub = nh.subscribe("/airframeState", 1, stateCallback);
	ros::Subscriber radioSub = nh.subscribe("/radioData", 1, radioCallback);
//	ros::Subscriber simSub = nh.subscribe("/simMode", 1, simModeCallback);

	ROS_INFO("flystixController : Spinning...");
	ros::spin();
	ROS_INFO("Deleting class instances.. ");
	delete RollPID;
	delete PitchPID;
	delete AltPID;
	ROS_INFO("Done deleting class instances");
}

void stateCallback(const fmMsgs::airframeState::ConstPtr& msg) {

	RollPID->setFeedback(msg->pose.x);
	PitchPID->setFeedback(msg->pose.y);
	AltPID->setFeedback(msg->alt);
	TurnPID->setFeedback(msg->incline);
	YawPID->setFeedback( fmod((msg->pose.z-HeadingDesired),M_PI) );

	state = *msg;

//	ROS_WARN("got R: %2.2f \t P: %2.2f \t A:%2.2f \n",RollPID->getFeedback(),PitchPID->getFeedback(), AltPID->getFeedback());

//	printf("Current pitch : %2.2f\n", msg->pose.y);
//	bankPtr->setFeedback(msg->)
}

void radioCallback(const fmMsgs::airframeControl::ConstPtr& msg) {
	static uint8_t mode = 10;
	/*
	 * msg->aileron_left => desired bank angle
	 * msg->throttle => desired airspeed
	 * msg->rudder => desired
	 * msg->mode => [0]automatic / [1]manual flight
	 */
	if (mode != msg->mode) { //We changed mode on the Radio transmitter
		RollPID->SetMode(msg->mode ? AUTOMATIC : MANUAL);
		PitchPID->SetMode(msg->mode ? AUTOMATIC : MANUAL);
		AltPID->SetMode(msg->mode ? AUTOMATIC : MANUAL);
		TurnPID->SetMode(msg->mode ? AUTOMATIC : MANUAL);
		mode = msg->mode;
	}

	if (msg->mode == AUTOMATIC) { // Invert elevator when in auto, otherwise it's weird..
		RollPID->setSetpoint(msg->aileron_left * D2R(75)); // PID controlled bank angle
//		RollPID->setSetpoint(YawPID->getOutput());
		PitchPID->setSetpoint(-msg->elevator * 0.7); // PID controlled pitch angle
//		PitchPID->setSetpoint(AltPID->getOutput()); // PID controlled (fixed)altitude
		TurnPID->setSetpoint(0);

	} else {
		RollPID->setSetpoint(msg->aileron_left);
		PitchPID->setSetpoint(msg->elevator);
		TurnPID->setSetpoint(msg->rudder);
	}
	radio = *msg;

}

void sendServo(const ros::TimerEvent& e) {
	fmMsgs::airframeControl msg = radio;

//	printf("altPID : %2.2f pitchPID : %2.2f alt %2.2f: \n", AltPID->getOutput(), PitchPID->getOutput(), AltPID->getFeedback());
//ROS_WARN("Pitch sp: %2.2f fb: %2.2f op:%2.2f", PitchPID->getSetpoint(), PitchPID->getFeedback(), PitchPID->getOutput());

	//RollPID->setSetpoint(0);
	//PitchPID->setSetpoint(0.2); // + AltPID->getOutput() );
	AltPID->setSetpoint(10.0);

	msg.stamp = ros::Time::now();
	msg.aileron_left = RollPID->getOutput();
	msg.aileron_right = RollPID->getOutput();
	msg.elevator = PitchPID->getOutput();
	msg.rudder = TurnPID->getOutput();
//	msg.elevator = PitchPID->getOutput() * cos(state.pose.x) - radio.rudder * sin(state.pose.x);
//	msg.rudder = PitchPID->getOutput() * sin(state.pose.x) + radio.rudder * cos(state.pose.x);
	msg.throttle = radio.throttle;

	controlPub.publish(msg);
}

//void simModeCallback(const fmMsgs::simData::ConstPtr& msg) {
//	static uint8_t simMode = 0;
//
//	if (simMode != msg->mode) {
//		RollPID->Initialize(RollPID->getSetpoint());
//		PitchPID->Initialize(PitchPID->getSetpoint());
//		ROS_INFO("Resetting...");
//		simMode = msg->mode;
//	}
//}


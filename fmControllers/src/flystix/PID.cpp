/*
 * PID.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: peter
 */

#include "PID.hpp"

PID::PID() {
	reset();
}
PID::PID(ros::NodeHandle* nh, ros::Rate updateRate) {
	reset();
	createTimerCallback(nh, updateRate);
}
PID::PID(double initP, double initI, double initD, double outputMax, ros::NodeHandle* nh,
         ros::Rate updateRate) {
	reset();
	P = initP;
	I = initI;
	D = initD;
	outMax = outputMax;
	createTimerCallback(nh, updateRate);
}
PID::PID(double initP, double initI, double initD, double outputMax) {
	reset();
	P = initP;
	I = initI;
	D = initD;
	outMax = outputMax;
}

double PID::update(void) {
	ros::Time t = ros::Time::now();
	update(t - stamp);
	stamp = t;
	return out;
}

double PID::update(ros::Duration dt) {
	static double _e;
	double e = setPoint - feedback;
	sum += e * dt.toSec();

	if (sum * I > outMax)
		sum = outMax / I;
	if (sum * I < -outMax)
		sum = -outMax / I;

	out = P * e + I * sum + D * (e - _e) / dt.toSec();

	_e = e;

	return out;
}

void PID::setSetPoint(double u) {
	setPoint = u;
}
double PID::getSetPoint(void) {
	return setPoint;
}
void PID::setFeedback(double z) {
	feedback = z;
}
double PID::output(void) {
	return out;
}
void PID::setGains(double newP, double newI, double newD) {
	P = newP;
	I = newI;
	D = newD;
}
void PID::setOutputMax(double newMax) {
	outMax = newMax;
}

void PID::timerCallback(const ros::TimerEvent& e) {
	update(e.current_real - stamp);
	stamp = e.current_real;
}

void PID::createTimerCallback(ros::NodeHandle* nh, ros::Rate rate) {
	timer = nh->createTimer(rate.expectedCycleTime(), &PID::timerCallback, this);
}

void PID::reset(void) {
	P = 0;
	I = 0;
	D = 0;
	outMax = 0; // Zero -> No limit...!
	setPoint = 0;
	feedback = 0;
	err = 0;
	sum = 0;
	stamp = ros::Time::now();
}

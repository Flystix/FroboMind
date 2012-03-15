/*
 * Yap yap
 */

#ifndef __PID_HPP__
#define __PID_HPP__

#include <ros/ros.h>

class PID {
public:
	PID();
	PID(ros::NodeHandle*, ros::Rate updateRate);
	PID(double initP, double initI, double initD, double outputMax, ros::NodeHandle*, ros::Rate updateRate);
	PID(double initP, double initI, double initD, double outputMax);
	void setSetPoint(double u);
	double getSetPoint(void);
	void setFeedback(double z);
	double output(void);
	void setGains(double newP, double newI, double newD);
	void setOutputMax(double newMax);
	double update(void);
	double update(ros::Duration dt);
private:
	void createTimerCallback(ros::NodeHandle* nh, ros::Rate r);
	void timerCallback(const ros::TimerEvent& e);
	void reset(void);
	double P, I, D;
	double setPoint, feedback;
	double err, sum, outMax;
	double out;
	ros::Timer timer;
	ros::Time stamp;
};

#endif

/*
 * kalman.h
 *
 *  Created on: Mar 20, 2012
 *      Author: peter
 */

#ifndef KALMAN_HPP_
#define KALMAN_HPP_

#include <math.h>
#include <semaphore.h>
#include <ros/ros.h>
#include <fmMsgs/accelerometer.h>
#include <fmMsgs/gyroscope.h>
#include <fmMsgs/magnetometer.h>
#include <fmMsgs/gps_state.h>
#include <fmMsgs/altitude.h>
#include <fmMsgs/airSpeed.h>
#include <fmMsgs/airframeState.h>
#include "ekfAttitude.hpp"
#include "ekfYaw.hpp"

class kalman{
public:
	kalman(ros::NodeHandle& nh, ros::NodeHandle& n);
	~kalman();
	void gyroCallback(const fmMsgs::gyroscope&);
	void accCallback(const fmMsgs::accelerometer&);
	void magCallback(const fmMsgs::magnetometer&);
	void gpsCallback(const fmMsgs::gps_state&);
	void altCallback(const fmMsgs::altitude&);
	void pitotCallback(const fmMsgs::airSpeed&);
	void pubCallback(const ros::TimerEvent&);
private:
	ros::Publisher state_pub;
	fmMsgs::airframeState state;
	sem_t attEstLock, yawEstLock;

	double wx, wy, wz;
	double temperature;
	double pressure;
	double levelLimit;
	double pitotOffset;

	ekfAtt* attitudeEstimator;
	ekfYaw* headingEstimator;

	ros::Timer pub_timer;
};

#endif /* KALMAN_HPP_ */

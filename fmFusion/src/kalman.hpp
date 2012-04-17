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
#include "ekfAttQuat.hpp"
#include "ekfYaw.hpp"
//#include "ekfPos.hpp"

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
	fmMsgs::airframeState* getState(void);
private:

	fmMsgs::airframeState state;
	ros::Publisher state_pub;
	sem_t attEstLock, yawEstLock, posEstLock;

	double wx, wy, wz;
	double temperature;
	double pressure;
	double pitotOffset;

//	ekfAtt* attitudeEstimator;
	ekfAttQuat* attitudeEstimator;
	ekfYaw* headingEstimator;

	double xAtt[4];
	double PAtt[16];
	double xYaw[1];
	double PYaw[1];

//	ekfPos* positionEstimator;

	ros::Timer pub_timer;
};

#endif /* KALMAN_HPP_ */

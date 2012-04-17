/*
 * itg3200.cpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */

#include <unistd.h>
#include <stdio.h>
#include <linux/types.h>
#include <semaphore.h>
#include <ros/ros.h>
#include "i2cfile.hpp"
#include "itg3200.hpp"

#define nInitSamples 100

itg3200::itg3200(i2cfile* i2c_ptr, ros::NodeHandle* nh_ptr, ros::Rate rate,	itg3200_callBackFunc dataReadyCallBack = 0) {
	char str[80];
	ROS_INFO("ITG3200 : Initializing");
	i2c = i2c_ptr;
	i2c->write_byte(addr, ITG3200_REG_POWER_MGMT, ITG3200_RESET);
	freq = 1 / rate.expectedCycleTime().toSec();

	fullscale = ITG3200_FULL_SCALE_2000;

	/* Setup bandwidth, in order to satisfy the Nyquist theorem*/
	if (freq > 512)
		low_pass = ITG3200_LP_256;
	else if (freq >= 376)
		low_pass = ITG3200_LP_188;
	else if (freq >= 196)
		low_pass = ITG3200_LP_98;
	else if (freq >= 84)
		low_pass = ITG3200_LP_42;
	else if (freq >= 40)
		low_pass = ITG3200_LP_20;
	else if (freq >= 20)
		low_pass = ITG3200_LP_10;
	else
		low_pass = ITG3200_LP_5;

	ROS_INFO("ITG3200 : LP/FS REG => 0x%02X", fullscale | low_pass);

	i2c->write_byte(addr, ITG3200_REG_LP_FULL_SCALE, fullscale | low_pass);
	powermanagement = i2c->read_byte(addr, ITG3200_REG_POWER_MGMT);
	bias[0] = 0;
	bias[1] = 0;
	bias[2] = 0;
	__u8 buf[6];
	for (int i = 0 ; i < nInitSamples ; i++) {
		i2c->read_block(addr, ITG3200_REG_GYRO_XOUT_H, buf, 6);
		bias[0] += (__u16)(buf[1] | buf[0] << 8);
		bias[1] += (__u16)(buf[3] | buf[2] << 8);
		bias[2] += (__u16)(buf[5] | buf[4] << 8);
		usleep(10000);
	}
	bias[0] = bias[0] / nInitSamples;
	bias[1] = bias[1] / nInitSamples;
	bias[2] = bias[2] / nInitSamples;

	sprintf(str, "\tBias : (x,y,z) = (%4i, %4i, %4i)", bias[0], bias[1], bias[2]);
	ROS_INFO("ITG3200 : %s", str);

	sem_init(&lock, 0, 1);
	nh = nh_ptr;
	timer = nh->createTimer(rate, &itg3200::timerCallback, this);
	dataCallback = dataReadyCallBack;
	sprintf(str, "\tSampling @ %2.2f Hz.", (float) 1.0f / rate.expectedCycleTime().toSec());
	ROS_INFO("ITG3200 : %s", str);
	ROS_INFO("ITG3200 : Initialization done!");
}

void itg3200::pull(void) {
	__u8 buf[6];
	if (i2c->read_block(addr, ITG3200_REG_GYRO_XOUT_H, buf, 6) == 6) {
		sem_wait(&lock);
		data[0] = (float)((__s16)(buf[3] | buf[2] << 8) - bias[1]) * - 0.001213527; // X-axis (rad / s)
		data[1] = (float)((__s16)(buf[1] | buf[0] << 8) - bias[0]) *   0.001213527; // Y-axis (rad / s)
		data[2] = (float)((__s16)(buf[5] | buf[4] << 8) - bias[2]) *   0.001213527; // Z-axis (rad / s)
		timeStamp = ros::Time::now();
		sem_post(&lock);
	}else
		ROS_WARN("ITG3200 : Error while reading data!");
}
void itg3200::getData(float (*dataOut)[3], ros::Time timeStampOut) {
	sem_wait(&lock);
	for (int i = 0 ; i < 3 ; i++)
		(*dataOut)[i] = data[i];
	timeStampOut = timeStamp;
	sem_post(&lock);
}
void itg3200::timerCallback(const ros::TimerEvent&) {
	pull();
	/* Make usr copy of data */
	fmMsgs::gyroscope myGyro;
	sem_wait(&lock);
	myGyro.vector.x = data[0];
	myGyro.vector.y = data[1];
	myGyro.vector.z = data[2];
	myGyro.stamp = timeStamp;
	sem_post(&lock);
	/* Run callback function*/
	(*dataCallback)(myGyro);
}

itg3200::~itg3200() {
	timer.stop();
}

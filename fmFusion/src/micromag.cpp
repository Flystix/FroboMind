/*
 * micromag.cpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */

#include "i2cfile.hpp"
#include "micromag.hpp"
#include <semaphore.h>
#include <ros/ros.h>

micromag::micromag(i2cfile* i2c_ptr, ros::NodeHandle* nh_ptr, ros::Rate rate,
                   micromag_callBackFunc dataReadyCallBack = 0, int windowSize = 32) {
	ROS_INFO("micromag : Initializing");
	char str[80];
	i2c = i2c_ptr;

	sem_init(&lock, 0, 1);

	double freq = 1 / rate.expectedCycleTime().toSec();

	setWindow(windowSize);

	nh = nh_ptr;
	timer = nh->createTimer(rate, &micromag::timerCallback, this);
	if (dataReadyCallBack)
		dataCallback = dataReadyCallBack;

	sprintf(str, "Sampling @ %2.2f Hz", freq);
	ROS_INFO("micromag : \t%s", str);
	ROS_INFO("micromag : Initialization done");

	// Probe device... Return 0 on failure..
}

micromag::~micromag() {
	timer.stop();
}

void micromag::pull(void) {
	uint8_t buf[12];
	int32_t data_buf[3];
	if (i2c->read_block(addr, MICROMAG_DAT_REG_X, buf, 12) == 12) {
		for (int i = 0; i < 3; i++) {
			data_buf[i] = ((buf[4 * i + 3] & 0xFF) << (3 * 8) |
					       (buf[4 * i + 2] & 0xFF) << (2 * 8) |
					       (buf[4 * i + 1] & 0xFF) << (1 * 8) |
					       (buf[4 * i + 0] & 0xFF) << (0 * 8));
		}
		sem_wait(&lock);
		data[0] = -scale * (float) data_buf[0];
		data[1] =  scale * (float) data_buf[1];
		data[2] = -scale * (float) data_buf[2];
		timeStamp = ros::Time::now();
		sem_post(&lock);
	} else
		ROS_WARN("micromag : Error while reading data!");
}

void micromag::getData(float(*dataOut)[3], ros::Time* timeStampOut) {
	sem_wait(&lock);
	for (int i = 0; i < 3; i++)
		(*dataOut)[i] = data[i];
	*timeStampOut = timeStamp;
	sem_post(&lock);
}

void micromag::getRawData(float (*dataOut)[3], ros::Time* timeStampOut) {
	uint8_t buf[6];
	int16_t data_buf[3];
	if (i2c->read_block(addr, 0x12, buf, 6) == 6) {
		for (int i = 0; i < 3; i++) {
			data_buf[i] = ((buf[2 * i + 1] & 0xFF) << (1 * 8) |
					       (buf[2 * i + 0] & 0xFF) << (0 * 8));
		}
		(*dataOut)[0] = -(float) data_buf[0];
		(*dataOut)[1] =  (float) data_buf[1];
		(*dataOut)[2] = -(float) data_buf[2];
		*timeStampOut = ros::Time::now();
	} else
		ROS_WARN("micromag : Error while reading data!");
}

void micromag::timerCallback(const ros::TimerEvent&) {
	pull();
	/* Make usr copy of data */
	fmMsgs::magnetometer myMag;
	sem_wait(&lock);
	myMag.vector.x = data[0];
	myMag.vector.y = data[1];
	myMag.vector.z = data[2];
	myMag.stamp = timeStamp;
	sem_post(&lock);
	/* Run callback function*/
	(*dataCallback)(myMag);
}

void micromag::setWindow(int window) {
	window_reg = window;
	i2c->write_byte(addr, MICROMAG_WINDOW_REG, window_reg);
	usleep(5000);
	window_reg = i2c->read_byte(addr, MICROMAG_WINDOW_REG);
	scale = baseScale / window_reg;
	ROS_INFO("micromag : Setting window %i", window_reg);
}

bool micromag::probe(i2cfile* i2c) {
	return (i2c->read_word(addr, 0x0C) == 0xAA88);
}

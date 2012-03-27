/*
 * hmc5883l.cpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */

#include "i2cfile.hpp"
#include "hmc5883l.hpp"
#include <semaphore.h>
#include <ros/ros.h>

hmc5883l::hmc5883l(i2cfile* i2c_ptr, ros::NodeHandle* nh_ptr, ros::Rate rate, hmc5883l_callBackFunc dataReadyCallBack = 0) {
	ROS_INFO("HMC5883L : Initializing");
	char str[80];
	i2c = i2c_ptr;
	sem_init(&lock, 0, 1);
	ctl_reg_a = (0x03 << HMC5883L_MA0) | (0x06 << HMC5883L_DO0) | (0x00 << HMC5883L_MS0);
	i2c->write_byte(addr, HMC5883L_CONF_REG_A, ctl_reg_a);

	mode_reg = (0x00 << HMC5883L_MD0);
	i2c->write_byte(addr, HMC5883L_MODE_REG, mode_reg);

	setScale(hmc5883l_scale_130);

	nh = nh_ptr;
	timer = nh->createTimer(rate, &hmc5883l::timerCallback, this);
	if (dataReadyCallBack)
		dataCallback = dataReadyCallBack;

	sprintf(str, "Sampling @ %2.2f Hz", (float) 1.0f / rate.expectedCycleTime().toSec());
	ROS_INFO("HMC5883L : \t%s", str);
	ROS_INFO("HMC5883L : Initialization done");
}

hmc5883l::~hmc5883l() {
	timer.stop();
}

void hmc5883l::pull(void) {
	__s8 buf[6];
	__s16 data_buf[3];
	if (i2c->read_block(addr, HMC5883L_DAT_REG_XH, buf, 6) == 6) {
		data_buf[0] = ((buf[0] & 0xFF) << 8 | (buf[1] & 0xFF));
		data_buf[1] = ((buf[4] & 0xFF) << 8 | (buf[5] & 0xFF));
		data_buf[2] = ((buf[2] & 0xFF) << 8 | (buf[3] & 0xFF));
		sem_wait(&lock);
		data[0] = scale * (float)data_buf[0];
		data[1] = scale * (float)data_buf[1];
		data[2] = scale * (float)data_buf[2];
		timeStamp = ros::Time::now();
		sem_post(&lock);
	}else
		ROS_WARN("HMC5883L : Error while reading data!");
}

void hmc5883l::getData(float (*dataOut)[3], ros::Time timeStampOut) {
	sem_wait(&lock);
	for (int i = 0 ; i < 3 ; i++)
		(*dataOut)[i] = data[i];
	timeStampOut = timeStamp;
	sem_post(&lock);
}

void hmc5883l::timerCallback(const ros::TimerEvent&) {
	float _data[3];
	ros::Time _timeStamp;
	pull();
	/* Make usr copy of data */
	sem_wait(&lock);
	for (int i = 0 ; i < 3 ; i++)
		_data[i] = data[i];
	_timeStamp = timeStamp;
	sem_post(&lock);
	/* Run callback function*/
	(*dataCallback)(&_data, _timeStamp);
}

void hmc5883l::setScale(int newScale) {
	switch (newScale) {
		case hmc5883l_scale_088:
			scale = (float)(1.0/1370.0);//0.1370;
			break;
		case hmc5883l_scale_130:
			scale = (float)(1.0/1090.0);//0.1090;
			break;
		case hmc5883l_scale_190:
			scale = (float)(1.0/820.0);//0.0820;
			break;
		case hmc5883l_scale_250:
			scale = (float)(1.0/660.0);
			break;
		case hmc5883l_scale_400:
			scale = (float)(1.0/440.0);
			break;
		case hmc5883l_scale_470:
			scale = (float)(1.0/390.0);
			break;
		case hmc5883l_scale_560:
			scale = (float)(1.0/330.0);
			break;
		case hmc5883l_scale_810:
			scale = (float)(1.0/230.0);
			break;
		}
	ctl_reg_b = (newScale << HMC5883L_GN0);
	i2c->write_byte(addr, HMC5883L_CONF_REG_B, ctl_reg_b);
}

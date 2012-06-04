/*
 * adxl345.cpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */
#include "adxl345.hpp"
#include "i2cfile.hpp"
#include <stdio.h>
#include <linux/types.h>
#include <semaphore.h>
#include <ros/ros.h>

adxl345::adxl345(i2cfile* i2c_ptr) {
	ROS_INFO("ADXL345 : Initialising...");
	sem_init(&lock, 0, 1);
	initi2c(i2c_ptr);
	ROS_INFO("ADXL345 : Initialization done!");
}

adxl345::adxl345(i2cfile* i2c_ptr, ros::NodeHandle* nh_ptr, ros::Rate rate, adxl_callBackFunc dataReadyCallBack = 0) {
	char str[16];
	ROS_INFO("ADXL345 : Initialising...");
	sem_init(&lock, 0, 1);
	freq = 1 / rate.expectedCycleTime().toSec();
	initi2c(i2c_ptr);
	ROS_INFO("ADXL345 : Setting up data callback...");
	setCallback(nh_ptr, rate, dataReadyCallBack);
	sprintf(str, "%2.2f Hz", (float) 1.0f / rate.expectedCycleTime().toSec());
	ROS_INFO("ADXL345 : Sampling @ %s", str);
	ROS_INFO("ADXL345 : Initialization done!");
}

adxl345::~adxl345(void) {
	pwct_reg |= (1 << 2); 	// Sleep (1 bit)
	pwct_reg &= ~(1 << 3);
	i2c->write_byte(addr, ADXL_PWCT_REG, pwct_reg);
	timer.stop();
}

void adxl345::initi2c(i2cfile* i2c_ptr) {
	char str[80];
	int check = 0;
	i2c = i2c_ptr;
//	check += i2c->write_byte(addr, ADXL_POWER_CTL, 1 << ADXL_PWCT_MEASURE);
//	check += i2c->write_byte(addr, ADXL_DATA_FORMAT, 1 << ADXL_FULL_RES);
//	check += i2c->write_byte(addr, ADXL_BW_RATE, 0x0F << 0);
//	check += i2c->write_byte(addr, ADXL_FIFO_CTL, 0x48 << 0);
	pwct_reg =    (0  << 0) |	// Wake up (2 bits)
				  (0  << 2) | 	// Sleep (1 bit)
				  (1  << 3) | 	// Measure (1 bit)
				  (0  << 4) | 	// Auto sleep (1 bit)
				  (0  << 5) ; 	// Link (1 bit)

	if (freq < 12.5)
		bw_rate = (7  << 0);
	else if(freq < 25)
		bw_rate = (8  << 0);
	else if(freq < 50)
		bw_rate = (9  << 0);
	else if(freq < 100)
		bw_rate = (10 << 0);
	else
		bw_rate = (11 << 0);

	bw_rate |= 	  (0  << 4) ; 	// Low power (1 bit)

	data_format = (3  << 0) | 	// Range (2 bits)
				  (0  << 2) | 	// Justify (1 bit)
				  (1  << 3) | 	// Full resolution (1 bit)
				  (0  << 4) | 	// N/A (1 bit)
				  (0  << 5) | 	// Invert interrupts (1 bit)
				  (0  << 6) | 	// 3-wire SPI (1 bit)
				  (0  << 7) ;	// Self-test (1 bit)

	fifo_ctl = 	  (0  << 0) |	// Samples (5 bits)
				  (0  << 5) | 	// Trigger (1 bit)
				  (0  << 6) ; 	// Fifo mode (2 bits) 0 = Bypass, 1 = FIFO, 2 = Stream, 3 = Trigger

	check = i2c->read_byte(addr, ADXL_DEVID_REG);
		if (check != ADXL_DEVID) {
			sprintf(str, "%02X, expected %02X", check, ADXL_DEVID);
			ROS_ERROR("ADXL345 : wrong device signature: %s", str);
		}
		check  = i2c->write_byte(addr, ADXL_PWCT_REG, pwct_reg);
		check += i2c->write_byte(addr, ADXL_DATA_FORMAT, data_format);
		check += i2c->write_byte(addr, ADXL_BW_RATE, bw_rate);
		check += i2c->write_byte(addr, ADXL_FIFO_CTL, fifo_ctl);
	if (check != 8)
		ROS_ERROR("ADXL345 : Init check sum mismatch!");
	else {
		ROS_INFO("ADXL35 : PWCT REG => %02X", pwct_reg);
		ROS_INFO("ADXL35 : DATA_FORMAT REG => %02X", data_format);
		ROS_INFO("ADXL35 : BW_RATE REG => %02X", bw_rate);
		ROS_INFO("ADXL35 : FIFO_CTL REG => %02X", fifo_ctl);
	}
}

void adxl345::setCallback(ros::NodeHandle* nh_ptr, ros::Rate rate, adxl_callBackFunc datareadyCallback) {
	nh = nh_ptr;
	dataCallback = datareadyCallback;
	timer = nh->createTimer(rate, &adxl345::timerCallback, this);
}

void adxl345::pull(void) {
	__s16 data_buf[3];
//	static const float a[3] = {0.9763500,  0.9725000,  1.0160100};
//	static const float b[3] = {0.0749097, -0.0060831, -0.0275528};
	static const float a[3] = {0.9763500*9.82,  0.9725000*9.82,  1.0160100*9.82}; /* m/s² */
	static const float b[3] = {0.0749097*9.82, -0.0060831*9.82, -0.0275528*9.82}; /* m/s² */
	ros::Time _timestamp = ros::Time::now();
	if (i2c->read_block(addr, ADXL_DATAX0, data_buf, 6) == 6) {
		sem_wait(&lock);
		data[0] = (float)((data_buf[0]) * -0.0039 * a[0] + b[0]);  // X-axis
		data[1] = (float)((data_buf[1]) * -0.0039 * a[1] + b[1]);  // Y-axis (Left -> right hand coor. sys.)
		data[2] = (float)((data_buf[2]) *  0.0039 * a[2] + b[2]);  // Z-axis
		timeStamp = _timestamp;
		sem_post(&lock);
	}else
		ROS_WARN("Error while reading ADXL345 data!");
}


void adxl345::getData(float (*dataOut)[3], ros::Time timeStampOut) {
	sem_wait(&lock);
	for (int i = 0 ; i < 3 ; i++)
		(*dataOut)[i] = data[i];
	timeStampOut = timeStamp;
	sem_post(&lock);
}

void adxl345::timerCallback(const ros::TimerEvent&) {
	fmMsgs::accelerometer msg;
	pull();
	/* Make usr copy of data */
	sem_wait(&lock);
	msg.vector.x = data[0];
	msg.vector.y = data[1];
	msg.vector.z = data[2];
	msg.stamp = timeStamp;
	sem_post(&lock);
	/* Run callback function*/
	if (dataCallback)
		(*dataCallback)(msg);
}

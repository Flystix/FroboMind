/*
 * MICROMAG.hpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */

#ifndef MICROMAG_HPP_
#define MICROMAG_HPP_

#include "i2cfile.hpp"
#include <semaphore.h>
#include <ros/ros.h>
#include <fmMsgs/magnetometer.h>

#define MICROMAG_I2C_ADDR       0x10
#define MICROMAG_IDENTITY       0xAA88

#define MICROMAG_DAT_REG_X      0x00
#define MICROMAG_DAT_REG_Y      0x04
#define MICROMAG_DAT_REG_Z      0x08
#define MICROMAG_DEVID_REG		0x0C
#define MICROMAG_REV_REG		0x0E
#define MICROMAG_WINDOW_REG     0x10

typedef void (*micromag_callBackFunc)(const fmMsgs::magnetometer&);

class micromag {
public:
	micromag(i2cfile*, ros::NodeHandle*, ros::Rate, micromag_callBackFunc, int);
	static bool probe(i2cfile*);
	~micromag(void);
	void pull();
	void getData(float (*)[3], ros::Time*);
	void getRawData(float (*)[3], ros::Time*);
	void timerCallback(const ros::TimerEvent&);
	void setWindow(int mode);
	static const __u8 addr = MICROMAG_I2C_ADDR;
private:
	sem_t lock;
	i2cfile *i2c;
	float data[3];
	ros::Time timeStamp;
	micromag_callBackFunc dataCallback;
	ros::Timer timer;
	ros::NodeHandle* nh;
	double scale;
	static const double baseScale = 0.002666667; /* Gauss / LSB */
	uint16_t window_reg;
};

#endif /* MICROMAG_HPP_ */

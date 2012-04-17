/*
 * hmc5883l.hpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */

#ifndef HMC5883L_HPP_
#define HMC5883L_HPP_

#include "i2cfile.hpp"
#include <semaphore.h>
#include <ros/ros.h>
#include <fmMsgs/magnetometer.h>

#define HMC5883L_I2C_ADDR       0x1E
#define HMC5883L_IDENTITY       0x00483433

#define HMC5883L_CONF_REG_A     0x00
#define HMC5883L_CONF_REG_B     0x01
#define HMC5883L_MODE_REG       0x02
#define HMC5883L_DAT_REG_XH     0x03
#define HMC5883L_DAT_REG_XL     0x04
#define HMC5883L_DAT_REG_ZH     0x05
#define HMC5883L_DAT_REG_ZL     0x06
#define HMC5883L_DAT_REG_YH     0x07
#define HMC5883L_DAT_REG_YL     0x08
#define HMC5883L_STAT_REG       0x09
#define HMC5883L_IDEN_REG_A     0x0A
#define HMC5883L_IDEN_REG_B     0x0B
#define HMC5883L_IDEN_REG_C     0x0C

// Control register A bits:
#define HMC5883L_MA1            6
#define HMC5883L_MA0            5
#define HMC5883L_DO2            4
#define HMC5883L_DO1            3
#define HMC5883L_DO0            2
#define HMC5883L_MS1            1
#define HMC5883L_MS0            0
// Control register B bits:
#define HMC5883L_GN2            7
#define HMC5883L_GN1            6
#define HMC5883L_GN0            5
// Mode reguster bits:
#define HMC5883L_MD0			0
#define HMC5883L_MD1			1

#define hmc5883l_scale_088		0
#define hmc5883l_scale_130		1
#define hmc5883l_scale_190		2
#define hmc5883l_scale_250		3
#define hmc5883l_scale_400		4
#define hmc5883l_scale_470		5
#define hmc5883l_scale_560		6
#define hmc5883l_scale_810		7

typedef void (*hmc5883l_callBackFunc)(const fmMsgs::magnetometer&);

class hmc5883l {
public:
	hmc5883l(i2cfile*, ros::NodeHandle*, ros::Rate, hmc5883l_callBackFunc);
	~hmc5883l(void);
	void pull();
	void getData(float (*)[3], ros::Time);
	void timerCallback(const ros::TimerEvent&);
	void setScale(int scale);
	static const __u8 addr = HMC5883L_I2C_ADDR;
private:
	sem_t lock;
	i2cfile *i2c;
	float data[3];
	ros::Time timeStamp;
	hmc5883l_callBackFunc dataCallback;
	ros::Timer timer;
	ros::NodeHandle* nh;
	float scale;
	float freq;
	int ctl_reg_a;
	int ctl_reg_b;
	int mode_reg;
};

#endif /* HMC5883L_HPP_ */

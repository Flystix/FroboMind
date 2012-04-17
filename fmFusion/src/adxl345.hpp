/*
 * adxl345.hpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */

#ifndef ADXL345_HPP_
#define ADXL345_HPP_

#include "i2cfile.hpp"
#include <fmMsgs/accelerometer.h>
#include <semaphore.h>
#include <ros/ros.h>

#define ADXL_ADDRESS			0x53	// ADXL345 slave address
#define ADXL_DEVID				0xE5	// 		- Device ID
#define ADXL_DEVID_REG	 		0x00 	// R	- Device ID register
#define ADXL_THRESH_TAP			0x1D 	// R/W 	- Tab Threshold
#define ADXL_OFSX				0x1E 	// R/W 	- X-axis offset
#define ADXL_OFSY				0x1F	// R/W 	- Y-axis offset
#define ADXL_OFSZ				0x20	// R/W 	- Z-axis offset
#define ADXL_DUR				0x21	// R/W 	- Tap duration
#define ADXL_LATENT				0x22	// R/W 	- Tap latency
#define ADXL_WINDOW				0x23	// R/W 	- Tap Window
#define ADXL_THRESH_ACT			0x24	// R/W 	- Activity threshold
#define ADXL_THRESH_INACT		0x25	// R/W 	- Inactivity threshold
#define ADXL_TIME_INACT  		0x26	// R/W 	- Inactivity time
#define ADXL_ACT_INACT_CTL  	0x27  	// R/W 	- Axis enable control for activity and inactivity detection
#define ADXL_THRESH_FF 			0x28	// R/W 	- Free-fall threshold
#define ADXL_TIME_FF  			0x29	// R/W 	- Free-fall time
#define ADXL_TAP_AXES 			0x2A	// R/W 	- Axis control for single tap/double tap
#define	ADXL_ACT_TAP_STATUS 	0x2B	// R  	- Source of single tap/double tap
#define ADXL_BW_RATE  			0x2C	// R/W	- Data rate and power mode control
#define ADXL_PWCT_REG  			0x2D	// R/W  - Power-saving features control
#define ADXL_PWCT_LINK			5
#define ADXL_PWCT_AUTO_SLEEP	4
#define ADXL_PWCT_MEASURE		3
#define ADXL_PWCT_SLEEP			2
#define ADXL_PWCT_WAKEUP_1		1
#define ADXL_PWCT_WAKEUP_0		0

#define ADXL_INT_ENABLE  		0x2E	// R/W  - Interrupt enable control
#define ADXL_INT_MAP  			0x2F	// R/W	- Interrupt mapping control
#define ADXL_INT_SOURCE  		0x30	// R  	- Source of interrupts
#define ADXL_INT_DATA_READY		7
#define ADXL_INT_SINGLE_TAB		6
#define ADXL_INT_DOUBLE_TAB		5
#define ADXL_INT_ACTIVITY		4
#define ADXL_INT_INACTIVITY		3
#define ADXL_INT_FREE_FALL		2
#define ADXL_INT_WATERMARK		1
#define ADXL_INT_OVERRUN		0SLAVE

#define ADXL_DATA_FORMAT  		0x31	// R/W  - Data format control
#define ADXL_SELF_TEST			7
#define ADXL_SPI				6		// 1 => 3 wire, 0 => 4 wire SPI
#define ADXL_INT_INV			5		// Invert interrupts
/*define N/A 					4*/
#define ADXL_FULL_RES			3
#define ADXL_JUSTIFY			2		// 1 => MSB first, 0 => LSB first
#define ADXL_RANGE_1			1		// 0 => +/- 2g, 1 => +/- 4g
#define ADXL_RANGE_0			0		// 2 => +/- 8g, 3 => +/- 16g

#define ADXL_DATAX0  			0x32	// R 	- X-Axis Data 0
#define ADXL_DATAX1  			0x33	// R  	- X-Axis Data 1
#define ADXL_DATAY0  			0x34	// R  	- Y-Axis Data 0
#define ADXL_DATAY1  			0x35	// R	- Y-Axis Data 1
#define ADXL_DATAZ0  			0x36 	// R  	- Z-Axis Data 0
#define ADXL_DATAZ1  			0x37	// R  	- Z-Axis Data 1
#define ADXL_FIFO_CTL  			0x38	// R/W 	- FIFO control
#define ADXL_FICT_MODE			6		// 0 => bypass, 1 => FIFO, 2 => Stream, 3 => Trigger
#define ADXL_FICT_TRIGGER		5
#define ADXL_FICT_SAMLPES		0		// Number of samples in FIFO. MAX = 31 (5 bit)

#define ADXL_FIFO_STATUS  		0x39	// R  	- FIFO status

typedef void (*adxl_callBackFunc)(const fmMsgs::accelerometer&);

class adxl345 {
public:
	adxl345(i2cfile*);
	adxl345(i2cfile*, ros::NodeHandle*, ros::Rate, adxl_callBackFunc);
	~adxl345();
	void pull();
	void getData(float (*)[3], ros::Time);
	void setCallback(ros::NodeHandle*, ros::Rate, adxl_callBackFunc);
	void timerCallback(const ros::TimerEvent&);
	static const __u8 addr = ADXL_ADDRESS;
private:
	void initi2c(i2cfile*);
	__u8 bw_rate, pwct_reg, data_format, fifo_ctl;
	double freq;
	sem_t lock;
	i2cfile *i2c;
	float data[3];
	ros::Time timeStamp;
	adxl_callBackFunc dataCallback;
	ros::NodeHandle* nh;
	ros::Timer timer;
};

#endif /* ADXL345_HPP_ */

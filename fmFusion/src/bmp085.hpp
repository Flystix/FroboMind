#ifndef BMP085_H_
#define BMP085_H_

#include "i2c-dev.h"
#include "i2cfile.hpp"
#include <ros/ros.h>
#include <errno.h>
#include <linux/string.h>
#include <fcntl.h>
#include <fmMsgs/altitude.h>

#define BMP085_I2C_ADDRESS              0x77
#define BMP085_CHIP_ID                  0x55

#define BMP085_CALIBRATION_DATA_START   0xAA
#define BMP085_CALIBRATION_DATA_LENGTH  11      /* 16 bit values */
#define BMP085_CHIP_ID_REG              0xD0
#define BMP085_VERSION_REG              0xD1
#define BMP085_CTRL_REG                 0xF4
#define BMP085_TEMP_MEASUREMENT         0x2E
#define BMP085_PRESSURE_MEASUREMENT     0x34
#define BMP085_CONVERSION_REGISTER_MSB  0xF6
#define BMP085_CONVERSION_REGISTER_LSB  0xF7
#define BMP085_CONVERSION_REGISTER_XLSB 0xF8
#define BMP085_TEMP_CONVERSION_TIME     5
#define BMP085_OSS						3	/* Oversampling setting = 0..3 */
#define BMP085_nSamples					8	/* = 2^BMP085 */

#define bmp085_requested_none	0x0000
#define bmp085_requested_temp	0x0010
#define bmp085_requested_pres	0x0020

typedef void (*bmp085_callBackFunc)(const fmMsgs::altitude&);

class bmp085 {
public:
	bmp085(i2cfile*, ros::NodeHandle*, ros::Rate, bmp085_callBackFunc);
	~bmp085();
	void pull();
	void timerCallback(const ros::TimerEvent&);
	static const __u8 addr = BMP085_I2C_ADDRESS;
private:
	void calc_temp(); /* Should only be called when lock is acquired !*/
	void calc_pres(); /* Should only be called when lock is acquired !*/
	void calc_alt(); /* Should only be called when lock is acquired !*/
	sem_t lock; /* Data acceess */
	i2cfile *i2c;

	ros::Time presTimeStamp;
	ros::Time tempTimeStamp;
		__u16 UT;	// Uncompensated temperature
//	unsigned long UT;
		__u32 UP;	// Uncompensated pressure
//	unsigned long UP;

	__s32 CT; // Temp [0.1°C]
	long CP; // Pressure [Pa]
	long P0; // Init. pressure [Pa]
	float alt; // Altitude above sea level [m]
	int got_pres;

	__u8 OverSamplingSetting;

	/* Data/time callback */
	bmp085_callBackFunc dataCallback;
	ros::Timer timer;
	ros::NodeHandle* nh;

	/* Calibration data */
	__s16 AC1, AC2, AC3;
	__u16 AC4, AC5, AC6;
	__s16 B1, B2;
	__s16 MB, MC, MD;

	/*  Calculated temperature correction coefficient */
	long B6;
};

#endif /* BMP085_H_ */

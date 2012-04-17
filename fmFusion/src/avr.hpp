/*
 * avr.hpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */

#ifndef AVR_HPP_
#define AVR_HPP_

#include "i2cfile.hpp"
#include <semaphore.h>
#include <fmMsgs/airframeControl.h>
#include <fmMsgs/airSpeed.h>
#include <fmMsgs/battery.h>
#include <ros/ros.h>

#define AVR_I2C_ADDRESS				0x08
#define AVR_REG_SERVOS				0x00
#define AVR_REG_SER_RIGHT_AIL		0x00
#define AVR_REG_SER_ELEVATOR		0x02
#define AVR_REG_SER_THROTTLE		0x04
#define AVR_REG_SER_RUDDER			0x06
#define AVR_REG_SER_LEFT_AIL		0x08
#define AVR_REG_SER_CONFIRM			0x0A
#define AVR_REG_RADIO				0x0C
#define AVR_REG_RAD_RIGHT_AIL		0x0C
#define AVR_REG_RAD_SWITCH			0x0E
#define AVR_REG_RAD_RUDDER			0x10
#define AVR_REG_RAD_THROTTLE		0x12
#define AVR_REG_RAD_ELEVATOR		0x14
#define AVR_REG_RAD_LEFT_AIL		0x16
#define AVR_REG_AIRSPEED			0x18
#define AVR_REG_BATTERY				0x1A
#define AVR_REG_RANGE				0x1C
#define AVR_REG_CRIT_BAT			0x1E
#define AVR_REG_LIMITS				0x20
#define AVR_REG_DEV_ID				0x3E
#define AVR_DEV_ID					0xA328
#define AVR_REG_DEV_VER				0x40
#define AVR_REG_CMD					0x42
#define AVR_CMD_LOAD_LIMITS			0x0001
#define AVR_CMD_SAVE_LIMITS			0x0002
#define AVR_CMD_RESET_LIMITS		0x0004
#define AVR_CMD_UPDATE_CENTERS		0x0008
#define AVR_CMD_UPDATE_LIMITS		0x0010
#define AVR_CMD_LOAD_CRITBAT		0x0020
#define AVR_CMD_SAVE_CRIT_BAT		0x0040

typedef void (*avr_callBackFunc)(const fmMsgs::airframeControl&,
								 const fmMsgs::airSpeed&,
								 const fmMsgs::battery&);

struct servo_limit_struct {
	__u16 max;
	__u16 center;
	__u16 min;
};

class avr{
public:
	avr(i2cfile*, ros::NodeHandle*, ros::Rate, avr_callBackFunc);
	~avr();

	void pull();									/* Update radio's and adc's */
	void timerCallback(const ros::TimerEvent&);		/* pull() and dataCallback() */
	void set_servos(const fmMsgs::airframeControl&);				/* convert to ±%fs -> µs, check limits and transfer */
	void set_servo(int channel, float value);		/* convert to ±%fs -> µs, check limits and transfer */
	void get_radios(float (*data)[6]);				/* convert to µs -> ±%fs, set and return*/
	float get_radio(int channel);					/* convert to µs -> ±%fs and return*/
	float get_airspeed(void);						/* convert to ? -> m/s and return */
	float get_battery_voltage(void);						/* convert mV -> V and return */
	float get_range();								/* convert mm -> m and return */
	void set_critical_voltage(float voltage);		/* convert V -> mV, transmit */
	float get_critical_voltage(void);				/* convert mV -> V, return */
	const static __u8 addr = AVR_I2C_ADDRESS;
private:
	int fs2us (float fs, struct servo_limit_struct* limit);		/* ±%fs -> µs */
	float us2fs (int us, struct servo_limit_struct* limit);		/* µs -> ±%fs */
	int chkLimits(int us, struct servo_limit_struct* limit); 	/* Return checked value */
	sem_t lock;			/* Data lock  */
	i2cfile *i2c;
	float data[3];
	ros::Time timeStamp;
	avr_callBackFunc dataCallback;
	ros::Timer timer;
	ros::NodeHandle* nh;

	__u16 servos[5];	/* 0x00, 0x02, 0x04, 0x06, 0x08 */
	__u16 confirm;		/* 0x0A */
	__u16 radios[5];	/* 0x0C, 0x10, 0x12, 0x14, 0x16 */
	__u16 man_switch;	/* 0x0E */
//	__u16 adc[3];		/* 0x18, 0x1A, 0x1C - Airspeed, battery, range */
	__u16 airspeed;
	__u16 battery;
	__u16 range;
	__u16 critical;		/* 0x1E */
	struct servo_limit_struct limits[5];
						/*			CH1	  CH2   CH3   CH4   CH5
						   max:		0x20, 0x26, 0x2C, 0x32, 0x38
						   center:	0x22, 0x28, 0x2E, 0x34, 0x3A
						   min:		0x24, 0x2A, 0x30, 0x36, 0x3C */
	__u16 dev_id;		/* 0x3E */
	__u16 dev_ver;		/* 0x40 */
	__u16 command;		/* 0x42 */
};

#endif /* AVR_HPP_ */

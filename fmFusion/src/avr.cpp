/*
 * avr.cpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */

#include "avr.hpp"
#include "i2cfile.hpp"
#include <stdio.h>
#include <linux/types.h>
#include <semaphore.h>
#include <ros/ros.h>

avr::avr(i2cfile* i2c_ptr, ros::NodeHandle* nh_ptr, ros::Rate rate,
		avr_callBackFunc dataReadyCallBack) {
	ROS_INFO("AVR : Initialising... ");
	char names[5][8] = { "L.Ail.", "Elev.", "Thro.", "Rudd.", "R.Ail." };
	char str[80];
	i2c = i2c_ptr;

	sem_init(&lock, 0, 1);

	dev_id = i2c->read_word(addr, AVR_REG_DEV_ID);
	if (dev_id != AVR_DEV_ID) {
		sprintf(str, "Error initialising AVR - wrong device signature: %04X, expected %04X", dev_id, AVR_DEV_ID);
		ROS_ERROR("AVR : %s", str);
	}

	dev_ver = i2c->read_word(addr, AVR_REG_DEV_VER);
	sprintf(str, "\tDevice version : %i.%i", (dev_ver & 0xFF00) >> 8, (dev_ver & 0x00FF) >> 0);
	ROS_INFO("AVR : %s", str);

	i2c->read_block(addr, AVR_REG_LIMITS, &limits, 30);
	ROS_INFO("AVR : \tServo limits: ");
	ROS_INFO("AVR : \t\tServo \tmax \tcenter \tmin :");
	for (int s = 0; s < 5; s++) {
		sprintf(str, "\t\t%s\t%i\t%i\t%i", names[s], limits[s].max, limits[s].center, limits[s].min);
		ROS_INFO("AVR : %s", str);
	}

	pull();

	critical = i2c->read_word(addr, AVR_REG_CRIT_BAT);
	sprintf(str, "\tCritical battery voltage is %2.2f", get_critical_voltage());
	ROS_INFO("AVR : %s", str);
	sprintf(str, "\tCurrent battery voltage is %2.2f.", get_battery_voltage());
	ROS_INFO("AVR : %s", str);

	nh = nh_ptr;
	timer = nh->createTimer(rate, &avr::timerCallback, this);
	dataCallback = dataReadyCallBack;
	sprintf(str, "%2.2f", (double) 1.0f / rate.expectedCycleTime().toSec());
	ROS_INFO("AVR : \tSampling @ %s", str);
	ROS_INFO("AVR : Initialization done!");
}

avr::~avr() {
	timer.stop();
}

void avr::pull(void) { /* Update radio's and adc's */
	__u8 buf[18];
	ros::Time _timeStamp = ros::Time::now();
	if (i2c->read_block(addr, AVR_REG_RADIO, buf, 18) == 18) {
		sem_wait(&lock);
		radios[0] =  (__u16) (buf[0] << 0) | (buf[1] << 8);
		man_switch = (__u16) (buf[2] << 0) | (buf[3] << 8);
		radios[1] =  (__u16) (buf[4] << 0) | (buf[5] << 8);
		radios[2] =  (__u16) (buf[6] << 0) | (buf[7] << 8);
		radios[3] =  (__u16) (buf[8] << 0) | (buf[9] << 8);
		radios[4] =  (__u16)(buf[10] << 0) | (buf[11] << 8);
		airspeed  =  (__u16)(buf[12] << 0) | (buf[13] << 8);
		battery   =  (__u16)(buf[14] << 0) | (buf[15] << 8);
		range     =  (__u16)(buf[16] << 0) | (buf[17] << 8);
		timeStamp = _timeStamp;
		sem_post(&lock);
	} else
		ROS_WARN("Error pulling radio's and adc's from AVR!\n");

}
void avr::timerCallback(const ros::TimerEvent&) { /* pull() and dataCallback() */
	pull();
	if (dataCallback) {
		int manOverrule = man_switch > 1500 ? 1 : 0;
		float vair = get_airspeed(); // Convert [?] -> [m/s]
		float range = get_range(); // Convert [mm] -> [m]
		float battery = get_battery_voltage(); // Convert [mV] -> [V]
		ros::Time _timestamp = timeStamp;
		float radio_floats[5];
		for (int i = 0; i < 5; i++)
			radio_floats[i] = us2fs(radios[i], &(limits[i]));

		(*dataCallback)(&radio_floats, manOverrule, vair, range, battery,
				_timestamp);
	}
}
void avr::set_servos(float(*data)[5]){ /* convert to ±%fs -> µs, check limits and transfer */
	__u16 pulse_width[5];
	float value;
	struct servo_limit_struct* lim;
	for (int i = 0 ; i < 5 ; i++) {
		value = (*data)[i];
		value = value >  1 ?  1 : value;
		value = value < -1 ? -1 : value;
		lim = &(limits[i]);
		pulse_width[i] = fs2us(value, lim);
		pulse_width[i] = chkLimits(pulse_width[i], lim);
	}
	sem_wait(&lock);
	for (int i = 0 ; i < 5 ; i++)
		servos[i] = pulse_width[i];
	i2c->write_block(addr, AVR_REG_SERVOS, pulse_width, 10);
	sem_post(&lock);
}
void avr::set_servo(int channel, float value){ /* convert to ±%fs -> µs, check limits and transfer */
	struct servo_limit_struct* lim = &(limits[channel]);
	value = value >  1 ?  1 : value;
	value = value < -1 ? -1 : value;
	int pulse_width = fs2us(value, lim);
	pulse_width = chkLimits(pulse_width, lim);
	sem_wait(&lock);
	servos[channel] = pulse_width;
	i2c->write_word(addr, AVR_REG_SERVOS + channel * 2, servos[channel]);
	sem_post(&lock);
}
void avr::get_radios(float(*data)[6]){ /* convert to µs -> ±%fs, set and return*/

}
float avr::get_radio(int channel){ /* convert to µs -> ±%fs and return*/
	return 0.0;
}
float avr::get_airspeed(void){ /* convert to ? -> m/s and return */
	float ret;
	sem_wait(&lock);
	ret = (float) airspeed;
	sem_post(&lock);
	return ret;
}
float avr::get_battery_voltage(void) { /* convert mV -> V and return */
	float ret;
	sem_wait(&lock);
	ret = (float) battery / 1000;
	sem_post(&lock);
	return ret;
}
float avr::get_range() { /* convert mm -> m and return */
	float ret;
	sem_wait(&lock);
	ret = range;
	sem_post(&lock);
	return ret;
}
void avr::set_critical_voltage(float voltage) { /* convert V -> mV, transmit */
	sem_wait(&lock);
	critical = voltage * 1000;
	i2c->write_word(addr, AVR_REG_CRIT_BAT, critical);
	sem_post(&lock);
}
float avr::get_critical_voltage(void) { /* convert mV -> V, return */
	float ret;
	sem_wait(&lock);
	critical = i2c->read_word(addr, AVR_REG_CRIT_BAT);
	ret = (float)critical / 1000;
	sem_post(&lock);
	return ret;
}

int avr::fs2us(float fs, struct servo_limit_struct* limit){ /* ±%fs -> µs */
	float ret = (float)limit->center;
	if (fs > 0)
		ret = (float)(limit->center + fs * (limit->max - limit->center));
	if (fs < 0)
		ret = (float)(limit->center - fs * (limit->min - limit->center));
	return (int)ret;
}
float avr::us2fs(int us, struct servo_limit_struct* limit) { /* µs -> ±%fs */
	float ret = 0.0;
	if (us == 0)
		return ret;
	if (us > limit->center)
		ret = (float)((float)(us - limit->center) / (float)(limit->max - limit->center));
	if (us < limit->center)
		ret = (float)((float)(us - limit->center) / (float)(limit->center - limit->min));
	return ret;
}
int avr::chkLimits(int us, struct servo_limit_struct* limit){ /* Return checked value */
	if (us < limit->min)
		us = limit->min;
	if (us > limit->max)
		us = limit->max;
	return us;
}

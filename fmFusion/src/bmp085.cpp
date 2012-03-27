/*
 * bmp085.cpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */

#include "bmp085.hpp"
#include "i2cfile.hpp"
#include <stdio.h>
#include <linux/types.h>
#include <semaphore.h>
#include <ros/ros.h>
#include <math.h>

#define nInitSamples 25

bmp085::bmp085(i2cfile* i2c_ptr, ros::NodeHandle* nh_ptr, ros::Rate rate,
		bmp085_callBackFunc dataReadyCallBack = 0) {
	__u8 buf[22];
	char str[80];
	int check = 0;

	ROS_INFO("BMP085 : Initializing...");
	i2c = i2c_ptr;
	sem_init(&lock, 0, 1);
	sem_wait(&lock);

	check += i2c->read_block(addr, BMP085_CALIBRATION_DATA_START, buf,
			BMP085_CALIBRATION_DATA_LENGTH * 2);
	if (check != 22)
			ROS_ERROR("BMP085 : Init check sum mismatch!\n");

	/*parameters AC1-AC6*/
	AC1 = ((buf[0] << 8) | (buf[1] << 0));
	AC2 = ((buf[2] << 8) | (buf[3] << 0));
	AC3 = ((buf[4] << 8) | (buf[5] << 0));
	AC4 = ((buf[6] << 8) | (buf[7] << 0));
	AC5 = ((buf[8] << 8) | (buf[9] << 0));
	AC6 = ((buf[10] << 8) | (buf[11] << 0));

	/*parameters B1,B2*/
	B1 = ((buf[12] << 8) | (buf[13] << 0));
	B2 = ((buf[14] << 8) | (buf[15] << 0));

	/*parameters MB,MC,MD*/
	MB = ((buf[16] << 8) | (buf[17] << 0));
	MC = ((buf[18] << 8) | (buf[19] << 0));
	MD = ((buf[20] << 8) | (buf[21] << 0));


	ROS_INFO("BMP085 : Calibration values:");
	sprintf(str, "\tAC1 : \t %+6d", AC1);
	ROS_INFO("BMP085 : %s", str);
	sprintf(str, "\tAC2 : \t %+6d", AC2);
	ROS_INFO("BMP085 : %s", str);
	sprintf(str, "\tAC3 : \t %+6d", AC3);
	ROS_INFO("BMP085 : %s", str);
	sprintf(str, "\tAC4 : \t +%5d", AC4);
	ROS_INFO("BMP085 : %s", str);
	sprintf(str, "\tAC6 : \t +%5d", AC6);
	ROS_INFO("BMP085 : %s", str);
	sprintf(str, "\tB1 : \t %+6d", B1);
	ROS_INFO("BMP085 : %s", str);
	sprintf(str, "\tB2 : \t %+6d", B2);
	ROS_INFO("BMP085 : %s", str);
	sprintf(str, "\tMB : \t %+6d", MB);
	ROS_INFO("BMP085 : %s", str);
	sprintf(str, "\tMC : \t %+6d", MC);
	ROS_INFO("BMP085 : %s", str);
	sprintf(str, "\tMD : \t %+6d", MD);
	ROS_INFO("BMP085 : %s", str);

	OverSamplingSetting = 0x03;

	sem_post(&lock);
	dataCallback = 0;
	P0 = 0;
	long _P0 = 0;
	got_pres = 0;
	for (int i = 0; i < nInitSamples;) {
		pull();
		if (got_pres) {
			_P0 += CP;
			i++;
			got_pres = 0;
		}
		usleep(50000);
	}

	P0 = _P0 / nInitSamples;

	sprintf(str, "Initial pressure : %2.2f [hPa]", (float)P0 / 100);
	ROS_INFO("BMP085 : %s", str);

	tempTimeStamp = ros::Time::now() - ros::Duration(1, 0);

	nh = nh_ptr;
	timer = nh->createTimer(rate, &bmp085::timerCallback, this);
	dataCallback = dataReadyCallBack;
	sprintf(str, "Sampling @ %2.2f Hz", (float) 1.0f / rate.expectedCycleTime().toSec());
	ROS_INFO("BMP085 : %s", str);
	ROS_INFO("BMP085 : Initializtion done!");
}

bmp085::~bmp085(void) {
	timer.stop();
}

void bmp085::pull(void) {
	static int requested = bmp085_requested_none;
	__u8 buf[4];

	/* Handle prev. data request - Temperature*/
	if (requested == bmp085_requested_temp) {
		// printf("Acquiring temperature... ");
		sem_wait(&lock);
		i2c->read_block(addr, 0xF6, buf, 2);
		UT = (__u16) (buf[0] << 8) | (buf[1] << 0);
		tempTimeStamp = ros::Time::now();
		calc_temp();
		sem_post(&lock);
		requested = bmp085_requested_none;
		// printf("Done!\n");
	}

	/* Handle prev. data request - Pressure*/
	if (requested == bmp085_requested_pres) {
		// printf("Acquiring pressure... ");
		i2c->read_block(addr, 0xF6, buf, 3);
		sem_wait(&lock);
		UP = (__u32)(((buf[0] << 16)|
						(buf[1] << 8)|
						(buf[2] << 0)) >> (8 - OverSamplingSetting));
		presTimeStamp = ros::Time::now();
		calc_pres();
		calc_alt();
		sem_post(&lock);

		if (dataCallback) {
			/* Run callback function*/
			// printf("Calling data callback...\n");
			/* Make usr copy of data */
			sem_wait(&lock);
			float _alt = alt;				 // Altitude in meters
			float _pres = (float)CP * 0.01; // Pressure in hPa
			float _temp = (float)CT * 0.1;	 // Temp in °C
			ros::Time _timeStamp = presTimeStamp;
			sem_post(&lock);
			(*dataCallback)(_alt, _pres, _temp, _timeStamp);
		}
		requested = bmp085_requested_none;
		got_pres = 1;
		// printf("Done!\n");
	}

	/* Request temp. reading every second*/
	if (tempTimeStamp + ros::Duration(1) <= ros::Time::now()) {
		// printf("Requesting temperature...");
		i2c->write_byte(addr, 0xF4, 0x2E);
		requested = bmp085_requested_temp;
		// printf("Done!\n");
	}

	/* Request pres. reading */
	if (requested == bmp085_requested_none) {
		// printf("Requesting pressure...");
		i2c->write_byte(addr, 0xF4, 0x34 + (OverSamplingSetting << 6));
		requested = bmp085_requested_pres;
		// printf("Done!\n");
	}
}

void bmp085::calc_temp(void) {
	long x1, x2;
	x1 = ((UT - AC6) * AC5) >> 15;
	x2 = (MC << 11) / (x1 + MD);
	B6 = x1 + x2 - 4000;
	CT = (x1 + x2 + 8) >> 4;
}

void bmp085::calc_pres(void) {
	long x1, x2, x3, b3;
	unsigned long b4, b7;
	long p;

	x1 = (((B6 * B6) >> 12) * B2) >> 11;
	x2 = (AC2 * B6) >> 11;
	x3 = x1 + x2;
	b3 = (((((long) AC1) * 4 + x3) << OverSamplingSetting) + 2) >> 2;
	x1 = (AC3 * B6) >> 13;
	x2 = (B1 * ((B6 * B6) >> 12)) >> 16;
	x3 = (x1 + x2 + 2) >> 2;
	b4 = (AC4 * (unsigned long) (x3 + 32768)) >> 15;
	b7 = ((unsigned long) UP - b3) * (50000 >> OverSamplingSetting);
	p = ((b7 < 0x80000000) ? ((b7 << 1) / b4) : ((b7 / b4) << 1));
	x1 = p >> 8;
	x1 = (x1 * x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;
	CP = p;
}

void bmp085::calc_alt(void) {
	if (P0 != 0)
		alt = 44330 * (1 - pow(((float)CP / (float)P0), (float) (1.0 / 5.255)));
}

void bmp085::timerCallback(const ros::TimerEvent&) {
	pull();
}

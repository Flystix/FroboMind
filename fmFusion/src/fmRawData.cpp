/*
 * Yap yap
 */
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <ros/ros.h>
#include <fmMsgs/airframeControl.h>
#include <fmMsgs/airframeState.h>
#include <fmMsgs/battery.h>
#include <fmMsgs/airSpeed.h>
#include <fmMsgs/altitude.h>
#include <fmMsgs/gyroscope.h>
#include <fmMsgs/accelerometer.h>
#include <fmMsgs/magnetometer.h>

#include "i2cfile.hpp"
#include "avr.hpp"
#include "adxl345.hpp"
#include "itg3200.hpp"
#include "hmc5883l.hpp"
#include "bmp085.hpp"
#include "kalman.hpp"
#include "micromag.hpp"

ros::Publisher accPublisher; // v avr     		-> fmMsgs::airframe_control /radioData
ros::Publisher gyroPublisher; // v avr  			-> fmMsgs::battery /batteryData
ros::Publisher pitotPublisher;
ros::Publisher rangePublisher;
ros::Publisher magPublisher;
ros::Publisher oldMagPublisher;

ros::Subscriber servoSubscriber; // v fmMsgs::airframeControl /servoData -> avr

avr* myAvr = 0;

void accDataCallback(float (*)[3], ros::Time);
void gyroDataCallback(float (*)[3], ros::Time);
void barDataCallback(float, float, float, ros::Time);
void avrDataCallback(float (*)[5], int, float, float, float, ros::Time);
void avrSetControlsCallback(const fmMsgs::airframeControl::ConstPtr&);
void magDataCallback(float (*)[3], ros::Time);
void oldMagDataCallback(float (*)[3], ros::Time);
micromag* myMag;

int main(int argc, char** argv) {
	ros::init(argc, argv, "i2cnode");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	double accRate, gyroRate, magRate, barRate, avrRate, pubRate;
	ROS_INFO("fmAirframe : Reading parameters...");
	n.param<double>("accRate", accRate, 100);
	n.param<double>("gyroRate", gyroRate, 100);
	n.param<double>("magRate", magRate, 50);
	n.param<double>("barRate", barRate, 25);
	n.param<double>("avrRate", avrRate, 50);
	n.param<double>("pubRate", pubRate, 50);

	ROS_INFO("fmAirframe : Advertising topics...");
	accPublisher = nh.advertise<fmMsgs::accelerometer>("/accData", 1);
	gyroPublisher = nh.advertise<fmMsgs::gyroscope>("/gyroData", 1);
	magPublisher = nh.advertise<fmMsgs::magnetometer>("/magData", 1);
	statePublisher = nh.advertise<fmMsgs::airframeState>("/airframeState", 1);
	radioPublisher = nh.advertise<fmMsgs::airframeControl>("/radioData", 1);
	batteryPublisher = nh.advertise<fmMsgs::battery>("/batteryData", 1);
	pitotPublisher = nh.advertise<fmMsgs::airSpeed>("/pitotData", 1);
	rangePublisher = nh.advertise<fmMsgs::altitude>("/altData", 1);

	i2cfile i2c(3);
	ROS_INFO("fmAirframe : Starting hardware interfaces...");
	avr myAVR(&i2c, &nh, ros::Rate(avrRate), &avrDataCallback);
	adxl345 myAcc(&i2c, &nh, ros::Rate(accRate), &accDataCallback);
	hmc5883l myMag(&i2c, &nh, ros::Rate(magRate), &magDataCallback);
	bmp085 myBar(&i2c, &nh, ros::Rate(barRate), &barDataCallback);
	itg3200 myGyro(&i2c, &nh, ros::Rate(gyroRate), &gyroDataCallback);
	myAvr = &myAVR;

	ROS_INFO("fmAirframe : Spinning...");
	ros::spin();

	return 0;
}

void avrSetControlsCallback(const fmMsgs::airframeControl::ConstPtr& msg) {
	float servos[5];

	if (!myAvr)
		return;
	servos[0] = msg->aileron_left;
	servos[1] = msg->rudder;
	servos[2] = msg->throttle;
	servos[3] = msg->elevator;
	servos[4] = msg->aileron_right;

	myAvr->set_servos(&servos);
}

void avrDataCallback(float (*radio)[5], int manOverrule, float vAir,
		float range, float battery, ros::Time timestamp) {
	static long nControl = 0;
	static float vAir_lp = 0, bat_lp = 0;
	fmMsgs::airframeControl control;
	fmMsgs::airSpeed airspeed;
	fmMsgs::battery bat;

	control.stamp = timestamp;

	control.aileron_left = (double) (*radio)[0];
	control.rudder = (double) (*radio)[1];
	control.throttle = (double) (*radio)[2];
	control.elevator = (double) (*radio)[3];
	control.aileron_right = (double) (*radio)[4];
	control.mode = manOverrule;
	if (control.throttle < 0)
		control.throttle = 0;
	radioPublisher.publish(control);

	vAir_lp = 0.99 * vAir_lp + 0.01 * vAir; //TODO fjern disse.
	airspeed.airspeed = vAir_lp;
	airspeed.stamp = timestamp;
	pitotPublisher.publish(airspeed);

	bat_lp = 0.99 * bat_lp + 0.01 * battery;
	if (!(nControl++ % 50)) { // Publish @ 1 Hz
		bat.voltage = bat_lp;
		bat.stateOfCharge = -1; // Un calculated, for now...
		bat.stamp = timestamp;
		batteryPublisher.publish(bat);
	}
}

void accDataCallback(float (*accData)[3], ros::Time timestamp) {

	fmMsgs::accelerometer myAccData;
	myAccData.vector.x = (double) (*accData)[0];
	myAccData.vector.y = (double) (*accData)[1];
	myAccData.vector.z = (double) (*accData)[2];
	myAccData.stamp = timestamp;

	accPublisher.publish(myAccData);
}

void gyroDataCallback(float (*gyroData)[3], ros::Time timestamp) {

	fmMsgs::gyroscope myGyroData;
	myGyroData.vector.x = (double) (*gyroData)[0];
	myGyroData.vector.y = (double) (*gyroData)[1];
	myGyroData.vector.z = (double) (*gyroData)[2];
	myGyroData.stamp = timestamp;

	gyroPublisher.publish(myGyroData);
}

void oldMagDataCallback(float (*magData)[3], ros::Time timestamp) {
	fmMsgs::magnetometer myMagData;
	myMagData.stamp = timestamp;
	myMagData.vector.x = (double) (*magData)[0];
	myMagData.vector.y = (double) (*magData)[1];
	myMagData.vector.z = (double) (*magData)[2];

	oldMagPublisher.publish(myMagData);
}

void magDataCallback(float (*magData)[3], ros::Time timestamp) {
	fmMsgs::magnetometer myMagData;
	myMagData.stamp = timestamp;
	myMagData.vector.x = (double) (*magData)[0];
	myMagData.vector.y = (double) (*magData)[1];
	myMagData.vector.z = (double) (*magData)[2];

	magPublisher.publish(myMagData);
}

void barDataCallback(float alt, float pres, float temp, ros::Time timestamp) {
	fmMsgs::altitude msg;
	msg.stamp = timestamp;
	msg.range = myAvr->get_range();
	msg.altitude = alt;
	msg.pressure = pres;
	msg.temperature = temp;

	rangePublisher.publish(msg);
}


#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <ros/ros.h>
#include <fmMsgs/magnetometer.h>

#include "i2cfile.hpp"
#include "hmc5883l.hpp"
#include "micromag.hpp"

void magDataCallback(float (*)[3], ros::Time);
void oldMagDataCallback(float (*)[3], ros::Time);
ros::Publisher  magPublisher;
ros::Publisher  oldMagPublisher;
micromag* myMag;

int main(int argc, char** argv) {
	ros::init(argc, argv, "i2cnode");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	double magRate;
	int magWindow;
	ROS_INFO("fmMag : Reading parameters...");
	n.param<double> ("magRate", magRate, 50);
	n.param<int> ("magWindow", magWindow, 64);

	ROS_INFO("fmMag : Advertising topics...");
	magPublisher = nh.advertise<fmMsgs::magnetometer>("/magData", 1);
	oldMagPublisher = nh.advertise<fmMsgs::magnetometer>("/oldMagData", 1);

	i2cfile i2c(3);
	ROS_INFO("fmMag : Starting hardware interfaces...");
	hmc5883l myOldMag(&i2c, &nh, ros::Rate(magRate), &oldMagDataCallback);
	myMag = new micromag(&i2c, &nh, ros::Rate(magRate), &magDataCallback, magWindow);

	ROS_INFO("fmMag : Spinning...");
	ros::spin();

	delete myMag;
	return 0;
}

void magDataCallback(float (*magData)[3], ros::Time timestamp) {
	fmMsgs::magnetometer myMagData;
	myMagData.stamp = timestamp;
	myMagData.vector.x = (double) (*magData)[0];
	myMagData.vector.y = (double) (*magData)[1];
	myMagData.vector.z = (double) (*magData)[2];

	magPublisher.publish(myMagData);
//	estimator->magCallback(myMagData);
}

void oldMagDataCallback(float (*magData)[3], ros::Time timestamp) {
	fmMsgs::magnetometer myMagData;
	myMagData.stamp = timestamp;
	myMagData.vector.x = (double) (*magData)[0];
	myMagData.vector.y = (double) (*magData)[1];
	myMagData.vector.z = (double) (*magData)[2];

	oldMagPublisher.publish(myMagData);
//	estimator->magCallback(myMagData);
}


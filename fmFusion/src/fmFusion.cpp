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
#include "micromag.hpp"
#include "bmp085.hpp"
#include "kalman.hpp"

#define SAMPLE_SENSORS 		0x01 // If 1 -> sample from i2c
#define SUBSCRIBE_SENSORS 	0x02 // If 1 -> subscribe to sensors
#define RUN_ESTIMATOR  		0x04 // If 1-> run estimator and publish state estimate 0-> dont...
#define PUBLISH_SENSORS    	0x08 // If 1-> publish raw sensor data, 0-> dont...


ros::Publisher radioPublisher;		/* Publish from avrDataCallback (if ...?)*/
ros::Publisher batteryPublisher;	/* Publish from avrDataCallback (if ...?)*/
ros::Publisher statePublisher;		/* Publish from pubCallback (if RUN_ESTIMATOR) */
ros::Publisher magPublisher;		/* Publish from magDataCallback (if PUBLISH_SENSORS) */
ros::Publisher accPublisher;		/* Publish from accDataCallback (if PUBLISH_SENSORS) */
ros::Publisher gyroPublisher;		/* Publish from gyroDataCallback (if PUBLISH_SENSORS) */
ros::Publisher airspeedPublisher;	/* Publish from airspeedDataCallback (if PUBLISH_SENSORS) */
ros::Publisher altPublisher;		/* Publish from altDataCallback (if PUBLISH_SENSORS)*/

ros::Subscriber gyroSubscriber;		/* Runs gyroDataCallback (if SUBSCRIBE_SENSORS) */
ros::Subscriber accSubscriber;		/* Runs accDataCallback (if SUBSCRIBE_SENSORS) */
ros::Subscriber magSubscriber;		/* Runs magDataCallback (if SUBSCRIBE_SENSORS) */
ros::Subscriber airspeedSubscriber;	/* Runs airspeedDataCallback (if SUBSCRIBE_SENSORS) */
ros::Subscriber altSubscriber;		/* Runs altDataCallback (if SUBSCRIBE_SENSORS) */
ros::Subscriber gpsSubscriber;		/* Runs gpsDataCallback (if RUN_ESTIMATOR) */

ros::Subscriber servoSubscriber;	/* Runs avrSetControlsCallback (if SAMPLE_SENSORS) */

avr* myAvr = 0;
i2cfile* i2c;
adxl345* myAcc;
micromag* myMag;
bmp085* myBar;
itg3200* myGyro;

kalman* estimator = 0;
static volatile int mode;

void gyroDataCallback(const fmMsgs::gyroscope&);
void accDataCallback(const fmMsgs::accelerometer&);
void magDataCallback(const fmMsgs::magnetometer&);
void altDataCallback(const fmMsgs::altitude&);
void gpsDataCallback(const fmMsgs::gps_state&);
void airspeedDataCallback(const fmMsgs::airSpeed&);
void avrDataCallback(const fmMsgs::airframeControl&,
		   	   	   	 const fmMsgs::airSpeed&,
		   	   	   	 const fmMsgs::battery&);
void avrSetControlsCallback(const fmMsgs::airframeControl&);
void pubCallback(const ros::TimerEvent&);

int main(int argc, char** argv) {
	ros::init(argc, argv, "i2cnode");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	ros::Timer pub_timer;

	std::string modestr;
	n.param<std::string>("mode", modestr, "est");

	if (!modestr.compare("raw")) {
		mode = SAMPLE_SENSORS | PUBLISH_SENSORS;
		ROS_WARN("Mode set RAW");
	} else if (!modestr.compare("flight")) {
		mode = SAMPLE_SENSORS | RUN_ESTIMATOR;
		ROS_WARN("Mode set FLIGHT");
	} else if (!modestr.compare("sim")) {
		mode = SUBSCRIBE_SENSORS | RUN_ESTIMATOR;
		ROS_WARN("Mode set SIM");
	} else if (!modestr.compare("all")){
		mode = SAMPLE_SENSORS | PUBLISH_SENSORS | RUN_ESTIMATOR;
		ROS_WARN("Something...");
	} else {
		ROS_WARN("Error parsing mode, defaulting to FLIGHT");
		mode = SAMPLE_SENSORS | RUN_ESTIMATOR;
	}

	ROS_INFO("fmAirframe : Advertising topics...");
	if (mode & RUN_ESTIMATOR) { /* Run state estimator, publish state estimate? */
		double pubRate;
		n.param<double>("pubRate", pubRate, 50);
		ROS_INFO("fmFusion : Starting estimator...");
		estimator = new kalman(nh, n);
		statePublisher = nh.advertise<fmMsgs::airframeState>("/airframeState", 1);
		pub_timer = nh.createTimer(ros::Duration(1 / pubRate),pubCallback);
		gpsSubscriber = nh.subscribe("/fmExtractors/gps_state_msg", 1, gpsDataCallback);
	}

	if (mode & PUBLISH_SENSORS) { /* Publish sensors? */
		ROS_INFO("fmFusion : Advertising sensor topics...");
		accPublisher = nh.advertise<fmMsgs::accelerometer>("/accData", 1);
		gyroPublisher = nh.advertise<fmMsgs::gyroscope>("/gyroData", 1);
		magPublisher = nh.advertise<fmMsgs::magnetometer>("/magData", 1);
		airspeedPublisher = nh.advertise<fmMsgs::airSpeed>("/pitotData", 1);
		altPublisher = nh.advertise<fmMsgs::altitude>("/altData", 1);
	}

	if (mode & SAMPLE_SENSORS) { /* Sampling : Sensor data is read from I2C bus */
		ROS_INFO("fmFusion : Starting hardware interfaces...");

		double accRate, gyroRate, magRate, barRate, avrRate;
		n.param<double>("accRate", accRate, 100);
		n.param<double>("gyroRate", gyroRate, 100);
		n.param<double>("magRate", magRate, 50);
		n.param<double>("barRate", barRate, 25);
		n.param<double>("avrRate", avrRate, 50);

		int magWindow;
		n.param<int>("magWindow", magWindow, 64);

		i2c    = new i2cfile(3);
		myGyro = new itg3200(i2c, &nh, ros::Rate(gyroRate), &gyroDataCallback);
		myAcc  = new adxl345(i2c, &nh, ros::Rate(accRate), &accDataCallback);
		myMag  = new micromag(i2c, &nh, ros::Rate(magRate), &magDataCallback, magWindow);
		myBar  = new bmp085(i2c, &nh, ros::Rate(barRate), &altDataCallback);
		myAvr  = new avr(i2c, &nh, ros::Rate(avrRate), &avrDataCallback);

		radioPublisher = nh.advertise<fmMsgs::airframeControl>("/radioData", 1);
		servoSubscriber = nh.subscribe("/servoData", 1, avrSetControlsCallback);
	}

	if (mode & SUBSCRIBE_SENSORS) { /* Simulating :  Sensor data is read from external rosbag */
		ROS_INFO("fmFusion : Subscribing to sensor topics");
		gyroSubscriber = nh.subscribe("gyroData", 1, gyroDataCallback);
		accSubscriber = nh.subscribe("/accData", 1, accDataCallback);
		magSubscriber = nh.subscribe("/magData", 1, magDataCallback);
		airspeedSubscriber = nh.subscribe("/pitotData", 1, airspeedDataCallback); /* fixme*/
		altSubscriber = nh.subscribe("/altData", 1, altDataCallback);
	}else{ // NOT simulating...
		batteryPublisher = nh.advertise<fmMsgs::battery>("/batteryData", 1);
	}

	ROS_INFO("fmFusion : Spinning...");
	ros::spin();

	ROS_INFO("fmFusion : Exiting...");
	delete myAvr;
	if (mode & RUN_ESTIMATOR) {
		delete estimator;
	}

	if (mode & SAMPLE_SENSORS) {
		delete myGyro;
		delete myBar;
		delete myMag;
		delete myAcc;
		delete myAvr;
		delete i2c;
	}

	ROS_INFO("fmFusion : Goodbye...");
	return 0;
}

void avrSetControlsCallback(const fmMsgs::airframeControl& msg) {
	if (mode & SAMPLE_SENSORS && myAvr)
		myAvr->set_servos(msg);
}

void avrDataCallback(const fmMsgs::airframeControl& control,
					 const fmMsgs::airSpeed& airspeed,
					 const fmMsgs::battery& bat) {
	static long nControl = 0;


	radioPublisher.publish(control);

	if (mode & RUN_ESTIMATOR && estimator)
		estimator->pitotCallback(airspeed);
	if (mode & PUBLISH_SENSORS && airspeedPublisher)
		airspeedPublisher.publish(airspeed);


	if(!(mode & SUBSCRIBE_SENSORS) && !(nControl++ % 50)) {
		batteryPublisher.publish(bat);
	}
}

void gyroDataCallback(const fmMsgs::gyroscope& myGyroData) {
	if (mode & RUN_ESTIMATOR && estimator)
		estimator->gyroCallback(myGyroData);
	if (mode & PUBLISH_SENSORS && gyroPublisher)
		gyroPublisher.publish(myGyroData);
}

void accDataCallback(const fmMsgs::accelerometer& myAccData) {
	if (mode & RUN_ESTIMATOR && estimator)
		estimator->accCallback(myAccData);
	if (mode & PUBLISH_SENSORS && accPublisher)
		accPublisher.publish(myAccData);
}


void magDataCallback(const fmMsgs::magnetometer& myMagData) {
	if (mode & RUN_ESTIMATOR && estimator)
		estimator->magCallback(myMagData);
	if (mode & PUBLISH_SENSORS && magPublisher)
		magPublisher.publish(myMagData);
}

void altDataCallback(const fmMsgs::altitude& myAltData) {
	fmMsgs::altitude msg = myAltData;
	if (myAvr)
		msg.range = myAvr->get_range();
	if (mode & RUN_ESTIMATOR && estimator)
		estimator->altCallback(msg);
	if (mode & PUBLISH_SENSORS && altPublisher)
		altPublisher.publish(msg);
}

void gpsDataCallback(const fmMsgs::gps_state& gpsData) {
	if (mode & RUN_ESTIMATOR && estimator)
		estimator->gpsCallback(gpsData);
}
void airspeedDataCallback(const fmMsgs::airSpeed& airSpeedData) {
	if (mode & RUN_ESTIMATOR && estimator)
		estimator->pitotCallback(airSpeedData);
	if (mode & PUBLISH_SENSORS && airspeedPublisher)
		airspeedPublisher.publish(airSpeedData);
}

void pubCallback(const ros::TimerEvent& e) {
	/* publish state */
	fmMsgs::airframeState state;
	if (mode & RUN_ESTIMATOR && estimator)
		state = *(estimator->getState());
	if (mode & RUN_ESTIMATOR && statePublisher)
		statePublisher.publish(state);
}

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
#define PUBLISH_SENSORS    		0x08 // If 1-> publish raw sensor data, 0-> dont...


ros::Publisher radioPublisher;
ros::Publisher batteryPublisher;
ros::Publisher statePublisher;
ros::Publisher magPublisher;
ros::Publisher oldMagPublisher;
ros::Publisher accPublisher;
ros::Publisher gyroPublisher;
ros::Publisher pitotPublisher;
ros::Publisher altPublisher;

ros::Subscriber radioSubscriber;
ros::Subscriber batterySubscriber;
ros::Subscriber gyroSubscriber;
ros::Subscriber accSubscriber;
ros::Subscriber magSubscriber;
ros::Subscriber pitotSubscriber;
ros::Subscriber rangeSubscriber;
ros::Subscriber gpsSubscriber;

ros::Subscriber servoSubscriber;

avr* myAvr;

kalman* estimator;
static volatile int mode;

void accDataCallback(const fmMsgs::accelerometer&);
void gyroDataCallback(const fmMsgs::gyroscope&);
void magDataCallback(const fmMsgs::magnetometer&);
void altDataCallback(const fmMsgs::altitude&);
void avrDataCallback(const fmMsgs::airframeControl&,
		   	   	   	 const fmMsgs::airSpeed&,
		   	   	   	 const fmMsgs::battery&);
void avrSetControlsCallback(const fmMsgs::airframeControl&);
void pubCallback(const ros::TimerEvent&);

void deleteme(const ros::TimerEvent& e){
	ROS_WARN("hello");
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "i2cnode");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	ros::Timer pub_timer;

	i2cfile* i2c;
	adxl345* myAcc;
	micromag* myMag;
	bmp085* myBar;
	itg3200* myGyro;

	double accRate, gyroRate, magRate, barRate, avrRate, pubRate;
	int magWindow;
	std::string modestr;
	ROS_INFO("fmAirframe : Reading parameters...");
	n.param<double>("accRate", accRate, 100);
	n.param<double>("gyroRate", gyroRate, 100);
	n.param<double>("magRate", magRate, 50);
	n.param<double>("barRate", barRate, 25);
	n.param<double>("avrRate", avrRate, 50);
	n.param<double>("pubRate", pubRate, 50);
	n.param<int>("magWindow", magWindow, 64);
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
		ROS_WARN("Somethuing...");
	} else {
		ROS_WARN("Error parsing mode, defaulting to FLIGHT");
		mode = SAMPLE_SENSORS | RUN_ESTIMATOR;
	}

	ROS_INFO("fmAirframe : Advertising topics...");
	if (mode & RUN_ESTIMATOR) {
		ROS_WARN("Starting estimator....");
		estimator = new kalman(nh, n);
		ROS_WARN("Advertising topic....");
		statePublisher = nh.advertise<fmMsgs::airframeState>("/airframeState", 1);
		ROS_WARN("Creating publisher callback....");
		pub_timer = nh.createTimer(ros::Duration(1 / pubRate),pubCallback,false,true);
		ROS_WARN("Done...");
	}

	if (mode & PUBLISH_SENSORS) {
		accPublisher = nh.advertise<fmMsgs::accelerometer>("/accData", 1);
		gyroPublisher = nh.advertise<fmMsgs::gyroscope>("/gyroData", 1);
		magPublisher = nh.advertise<fmMsgs::magnetometer>("/magData", 1);
		oldMagPublisher = nh.advertise<fmMsgs::magnetometer>("/oldMagData", 1);
		pitotPublisher = nh.advertise<fmMsgs::airSpeed>("/pitotData", 1);
		altPublisher = nh.advertise<fmMsgs::altitude>("/altData", 1);
	}

	if (mode & SAMPLE_SENSORS) {
		i2c = new i2cfile(3);
		ROS_INFO("fmFusion : Starting hardware interfaces...");
		myAvr = new avr(i2c, &nh, ros::Rate(avrRate), &avrDataCallback);
		myAcc = new adxl345(i2c, &nh, ros::Rate(accRate), &accDataCallback);
		myMag = new micromag(i2c, &nh, ros::Rate(magRate), &magDataCallback, magWindow);
		myBar = new bmp085(i2c, &nh, ros::Rate(barRate), &altDataCallback);
		myGyro = new itg3200(i2c, &nh, ros::Rate(gyroRate), &gyroDataCallback);
	}

	if (mode & SUBSCRIBE_SENSORS) {

	}else{ // NOT simulating...
		batteryPublisher = nh.advertise<fmMsgs::battery>("/batteryData", 1);
	}
	servoSubscriber = nh.subscribe("/servoData", 1, avrSetControlsCallback);
	radioPublisher = nh.advertise<fmMsgs::airframeControl>("/radioData", 1);



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
	float servos[5];

	if (!myAvr)
		return;
//	servos[0] = msg->aileron_left;
//	servos[1] = msg->rudder;
//	servos[2] = msg->throttle;
//	servos[3] = msg->elevator;
//	servos[4] = msg->aileron_right;

	myAvr->set_servos(msg);
}

void avrDataCallback(const fmMsgs::airframeControl& control,
					 const fmMsgs::airSpeed& airspeed,
					 const fmMsgs::battery& bat) {
	static long nControl = 0;
	static float bat_lp = 0;


	radioPublisher.publish(control);

	if (mode & RUN_ESTIMATOR)
		estimator->pitotCallback(airspeed);
	if (mode & PUBLISH_SENSORS)
		pitotPublisher.publish(airspeed);


	if(!(mode & SUBSCRIBE_SENSORS) && !(nControl++ % 50)) {
		batteryPublisher.publish(bat);
	}
}

void accDataCallback(const fmMsgs::accelerometer& myAccData) {
	if (mode & RUN_ESTIMATOR)
		estimator->accCallback(myAccData);
	if (mode & PUBLISH_SENSORS)
		accPublisher.publish(myAccData);
}

void gyroDataCallback(const fmMsgs::gyroscope& myGyroData) {
	if (mode & RUN_ESTIMATOR)
		estimator->gyroCallback(myGyroData);
	if (mode & PUBLISH_SENSORS)
		gyroPublisher.publish(myGyroData);
}

void magDataCallback(const fmMsgs::magnetometer& myMagData) {
	if (mode & RUN_ESTIMATOR)
		estimator->magCallback(myMagData);
	if (mode & PUBLISH_SENSORS)
		magPublisher.publish(myMagData);
}

void altDataCallback(const fmMsgs::altitude& myAltData) {
	fmMsgs::altitude msg = myAltData;
	msg.range = myAvr->get_range();

	if (mode & RUN_ESTIMATOR)
		estimator->altCallback(msg);
	if (mode & PUBLISH_SENSORS)
		altPublisher.publish(msg);
}

void pubCallback(const ros::TimerEvent& e) {
	/* publish state */
	static uint32_t seq = 0;
	fmMsgs::airframeState state = *(estimator->getState());
	state.header.stamp = e.current_real;
	state.header.seq = seq++;
	statePublisher.publish(state);
}

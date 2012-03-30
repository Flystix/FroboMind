#include <math.h>
#include <semaphore.h>
#include <ros/ros.h>
#include <fmMsgs/accelerometer.h>
#include <fmMsgs/gyroscope.h>
#include <fmMsgs/magnetometer.h>
#include <fmMsgs/gps_state.h>
#include <fmMsgs/altitude.h>
#include <fmMsgs/airSpeed.h>
#include <fmMsgs/airframeState.h>
#include <fmMsgs/simData.h>
#include "kalman.hpp"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#define RAND(x) (*nd)()*x


boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >* nd;

void gyroCallback(const fmMsgs::gyroscope::ConstPtr&);
void accCallback(const fmMsgs::accelerometer::ConstPtr&);
void magCallback(const fmMsgs::magnetometer::ConstPtr&);
void gpsCallback(const fmMsgs::gps_state::ConstPtr&);
void altCallback(const fmMsgs::altitude::ConstPtr&);
void pitotCallback(const fmMsgs::airSpeed::ConstPtr&);
void simCallback(const fmMsgs::simData::ConstPtr&);
void pubCallback(const ros::TimerEvent&);

ros::Publisher state_pub;
fmMsgs::airframeState state;

double wx = 0, wy = 0, wz = 0;
double temperature = 25;
double pressure = 1013.25;
double levelLimit = 0;

double accNoise, gyrNoise, magNoise, altNoise, gpsNoise;

kalman* estPtr;

int main(int argc, char** argv) {
	ros::init(argc, argv, "flyStixKalman");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	boost::mt19937 rng;
	boost::normal_distribution<> normdist(0.0, 1.0);
	nd = new boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >(rng, normdist);

	state.Wn = 0;
	state.We = 0;

	double accVar, gyroVar, magVar, pubRate;
	n.param<double> ("accVar", accVar, 1);
	n.param<double> ("gyroVar", gyroVar, 1);
	n.param<double> ("magVar", magVar, 1);
	n.param<double> ("pubRate", pubRate, 50);
	n.param<double> ("isLevelThreshold", levelLimit, 15);

	n.param<double> ("accNoise", accNoise,0);
	n.param<double> ("gyrNoise", gyrNoise,0);
	n.param<double> ("magNoise", magNoise,0);
	n.param<double> ("altNoise", altNoise,0);
	n.param<double> ("gpsNoise", gpsNoise,0);

	levelLimit = levelLimit * M_PI / 180;

	estPtr = new kalman(nh, n);

	state.header.frame_id = "AirFrame";
	state.header.seq = 0;
	ROS_INFO("fmEstimator : Subscribing to topics");
	ros::Subscriber gyro_sub = nh.subscribe("/gyroData", 1, gyroCallback);
	ros::Subscriber acc_sub  = nh.subscribe("/accData", 1, accCallback);
	ros::Subscriber mag_sub  = nh.subscribe("/magData", 1, magCallback);
	ros::Subscriber gps_sub  = nh.subscribe("/fmExtractors/gps_state_msg", 1, gpsCallback);
	ros::Subscriber alt_sub  = nh.subscribe("/altData", 1, altCallback);
	ros::Subscriber pit_sub  = nh.subscribe("/pitotData", 1, pitotCallback);
	ros::Subscriber sim_sub  = nh.subscribe("/simMode", 1, simCallback);

	state_pub = nh.advertise<fmMsgs::airframeState>("/airframeState", 1);
	ros::Timer pub_timer = nh.createTimer(ros::Duration(1/pubRate), pubCallback);

	ROS_INFO("fmEstimator : Spinning...");
	ros::spin();

	delete estPtr;
}

void pubCallback(const ros::TimerEvent& e) {
//	ROS_WARN("Publishing...");
	/* publish state */
	static uint32_t seq = 0;
	state = *(estPtr->getState());
	state.header.stamp = ros::Time::now();
	state.header.seq = seq++;
	state_pub.publish(state);
}

void gyroCallback(const fmMsgs::gyroscope::ConstPtr& msg) {
	fmMsgs::gyroscope myMsg = *msg;
	myMsg.vector.x += RAND(gyrNoise);
	myMsg.vector.y += RAND(gyrNoise);
	myMsg.vector.z += RAND(gyrNoise);
	estPtr->gyroCallback(myMsg);
}

void accCallback(const fmMsgs::accelerometer::ConstPtr& msg) {
	fmMsgs::accelerometer myMsg = *msg;
	myMsg.vector.x += RAND(accNoise);
	myMsg.vector.y += RAND(accNoise);
	myMsg.vector.z += RAND(accNoise);
	estPtr->accCallback(myMsg);
}

void magCallback(const fmMsgs::magnetometer::ConstPtr& msg) {
	fmMsgs::magnetometer myMsg = *msg;
	myMsg.vector.x += RAND(magNoise);
	myMsg.vector.y += RAND(magNoise);
	myMsg.vector.z += RAND(magNoise);
	estPtr->magCallback(myMsg);
}

void gpsCallback(const fmMsgs::gps_state::ConstPtr& msg) {
	fmMsgs::gps_state myMsg = *msg;
	myMsg.alt += RAND(gpsNoise*10);
	myMsg.lon += RAND(gpsNoise);
	myMsg.lat += RAND(gpsNoise);
	myMsg.utm_e += RAND(gpsNoise);
	myMsg.utm_n += RAND(gpsNoise);
	estPtr->gpsCallback(myMsg);
}

void altCallback(const fmMsgs::altitude::ConstPtr& msg) {
	estPtr->altCallback(*msg);
}

void pitotCallback(const fmMsgs::airSpeed::ConstPtr& msg) {
	estPtr->pitotCallback(*msg);
}

void simCallback(const fmMsgs::simData::ConstPtr&) {
	estPtr->resetAtt(0,0);
	estPtr->resetYaw(0);
	estPtr->resetPos(0,0);
}

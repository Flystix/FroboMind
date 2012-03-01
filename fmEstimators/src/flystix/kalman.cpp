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
#include "ekfAttitude.hpp"
#include "ekfYaw.hpp"

void gyroCallback(const fmMsgs::gyroscope::ConstPtr&);
void accCallback(const fmMsgs::accelerometer::ConstPtr&);
void magCallback(const fmMsgs::magnetometer::ConstPtr&);
void gpsCallback(const fmMsgs::gps_state::ConstPtr&);
void altCallback(const fmMsgs::altitude::ConstPtr&);
void pitotCallback(const fmMsgs::airSpeed::ConstPtr&);
void pubCallback(const ros::TimerEvent&);

ros::Publisher state_pub;
fmMsgs::airframeState state;
sem_t attEstLock, yawEstLock;

double wx = 0, wy = 0, wz = 0;
double levelLimit = 0;

ekfYaw* yawEstimatorPtr;
ekfAttitude* attitudeEstimatorPtr;

int main(int argc, char** argv) {
	ros::init(argc, argv, "flyStixKalman");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	sem_init(&attEstLock, 0, 1);
	sem_init(&yawEstLock, 0, 1);

	double accVar, gyroVar, magVar;
	n.param<double> ("accVar", accVar, 1);
	n.param<double> ("gyroVar", gyroVar, 1);
	n.param<double> ("magVar", magVar, 1);
	n.param<double> ("isLevelThreshold", levelLimit, 15);
	levelLimit = levelLimit * M_PI / 180;

	/* INIT EXTENDED KALMAN FILTER FOR PITCH / ROLL*/
	ROS_INFO("fmEstimator : Initializing pitch/roll ekf...");
	const unsigned n_pr = 2;	//nb states
	const unsigned m_pr = 3;	//nb measures
	static const double _P0_pr[] = {1.0, 0.0, 0.0, 1.0};
	ekfAttitude::Vector x_pr(n_pr);
	ekfAttitude::Matrix P0_pr(n_pr, n_pr, _P0_pr);
	ekfAttitude::Vector z_pr(m_pr);
	z_pr(1) = 0;
	z_pr(2) = 0;
	z_pr(3) = 0;
	ekfAttitude::Vector u_pr(m_pr);
	x_pr(1) = 0;
	x_pr(2) = 0;

	ekfAttitude attitudeEstimator(gyroVar, accVar);
	attitudeEstimator.init(x_pr, P0_pr);
	attitudeEstimatorPtr = &attitudeEstimator;
	ROS_INFO("fmEstimator : Done");

	/* INIT EXTENDED KALMAN FILTER FOR YAW */
	ROS_INFO("fmEstimator : Initializing yaw ekf...");

	static const double _P0_y[] = {1.0};
	ekfYaw::Vector x_y(1);
	ekfYaw::Matrix P0_y(1, 1, _P0_y);
	ekfYaw::Vector z_y(3);
	z_y(1) = 0;
	z_y(2) = 0;
	z_y(3) = 0;
	ekfAttitude::Vector u_y(4);
	x_y(1) = 0;

	ekfYaw yawEstimator(gyroVar, magVar);
	yawEstimator.init(x_y, P0_y);
	yawEstimatorPtr = &yawEstimator;
	ROS_INFO("fmEstimator : Done");


	state.header.frame_id = "AirFrame";
	state.header.seq = 0;
	ROS_INFO("fmEstimator : Subscribing to topics");
	ros::Subscriber gyro_sub = nh.subscribe("/gyroData", 1, gyroCallback);
	ros::Subscriber acc_sub  = nh.subscribe("/accData", 1, accCallback);
	ros::Subscriber mag_sub  = nh.subscribe("/magData", 1, magCallback);
	ros::Subscriber gps_sub  = nh.subscribe("/fmExtractors/gps_state_msg", 1, gpsCallback);
	ros::Subscriber alt_sub  = nh.subscribe("/altData", 1, altCallback);
	ros::Subscriber pit_sub  = nh.subscribe("/pitotData", 1, pitotCallback);

	state_pub = nh.advertise<fmMsgs::airframeState>("/airframeState", 1);
	ros::Timer pub_timer = nh.createTimer(ros::Duration(0.1), pubCallback);
	ROS_INFO("fmEstimator : Spinning...");
	ros::spin();
}

void pubCallback(const ros::TimerEvent& e) {
	/* publish state */
	state.header.stamp = ros::Time::now();
	state.header.seq++;
	state_pub.publish(state);
}

void gyroCallback(const fmMsgs::gyroscope::ConstPtr& msg) {
	static ros::Time _stamp = ros::Time::now();
	double dt = (msg->stamp - _stamp).toSec();
	_stamp = msg->stamp;
	ekfAttitude::Vector uAtt(3);
	ekfAttitude::Vector xAtt(2);
	ekfYaw::Vector uYaw(2);
	ekfYaw::Vector xYaw(1);

	wx = msg->vector.x;
	wy = msg->vector.y;
	wz = msg->vector.z;

	/* Do attitude timeUpdate : */
	uAtt(1) = wx * dt;
	uAtt(2) = wy * dt;
	uAtt(3) = wz * dt;

	sem_wait(&attEstLock);
	attitudeEstimatorPtr->timeUpdateStep(uAtt);
	sem_post(&attEstLock);
	xAtt = attitudeEstimatorPtr->getX();

	/* Do yaw timeUpdate : */
	uYaw(1) = wy * dt;
	uYaw(2) = wz * dt;

	sem_wait(&yawEstLock);
	yawEstimatorPtr->updateAttitude(state.pose.x, state.pose.y);
	yawEstimatorPtr->timeUpdateStep(uYaw);
	sem_post(&yawEstLock);
	xYaw = yawEstimatorPtr->getX();

	/* Save back to airframe state */
	ROS_DEBUG("GYRO : x = : %2.2f, %2.2f\n", xAtt(1), xAtt(2));
	state.pose.x = xAtt(1);
	state.pose.y = xAtt(2);
	state.pose.z = xYaw(1);
}

void accCallback(const fmMsgs::accelerometer::ConstPtr& msg) {
	ekfAttitude::Vector z(3);
	ekfAttitude::Vector x(2);
	z(1) = msg->vector.x / 9.82;
	z(2) = msg->vector.y / 9.82;
	z(3) = msg->vector.z / 9.82;

	double v = asin(msg->vector.y / sqrt(pow(msg->vector.y,2) + pow(msg->vector.z,2)));
	state.incline = v * 0.1 + state.incline * 0.9;

	sem_wait(&attEstLock);
	attitudeEstimatorPtr->measureUpdateStep(z);
	sem_post(&attEstLock);
	x = attitudeEstimatorPtr->getX();

	state.pose.x = x(1);
	state.pose.y = x(2);
}

void magCallback(const fmMsgs::magnetometer::ConstPtr& msg) {
	// Run measure-update on stage 2
	ekfYaw::Vector z(3);
	ekfYaw::Vector x(1);

	/* Discard magnetometer data, if flight isn't some-what level*/
	if(state.pose.x > levelLimit || state.pose.x < -levelLimit ||
       state.pose.y > levelLimit || state.pose.y < -levelLimit)
		return;

	/* ALTERNATIVE (And more politically correct):
	 * Manipulate magnetometer confidence / covariance
	 * according to of-levelness...
	 */
	z(1) = msg->vector.x;
	z(2) = msg->vector.y;
	z(3) = msg->vector.z;
	sem_wait(&yawEstLock);
	yawEstimatorPtr->updateAttitude(state.pose.x, state.pose.y);
	yawEstimatorPtr->measureUpdateStep(z);
	sem_post(&yawEstLock);
	x = yawEstimatorPtr->getX();
	/* Update state */
	state.pose.z = x(1);
}

void gpsCallback(const fmMsgs::gps_state::ConstPtr& msg) {
	/* Run measure-update on stage 3 */
    /* msg->header;
     * msg->time_recv;
     * msg->time;
     * msg->lat;
     * msg->lon;
     * msg->utm_zone_num;
     * msg->utm_zone_let;
     * msg->utm_n;
     * msg->utm_e;
     * msg->alt;
     * msg->fix;
     * msg->sat;
     * msg->hdop;
     * msg->geoid_height; */
    /* Update state */
    state.lat = msg->lat;
    state.lon = msg->lon;
}

void altCallback(const fmMsgs::altitude::ConstPtr& msg) {
	/* Update altitude and dist. to ground */
    /* msg->range;
     * msg->altitude;
     * msg->pressure;
     * msg->temperature;
     * msg->stamp; */
    /* Update state */
	state.alt = msg->altitude;
}

void pitotCallback(const fmMsgs::airSpeed::ConstPtr& msg) {
	/* Update vAir */
	/* 	msg->airspeed;
	 * msg->stamp; */
	/* Update state */
	state.airspeed = msg->airspeed;
}


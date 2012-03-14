#include <math.h>
#include <ros/ros.h>
#include <fmMsgs/accelerometer.h>
#include <fmMsgs/gyroscope.h>
#include <fmMsgs/magnetometer.h>
#include <fmMsgs/gps_state.h>
#include <fmMsgs/altitude.h>
#include <fmMsgs/airSpeed.h>
#include <fmMsgs/airframeState.h>
#include "ekfAtt.hpp"
#include "ekfYaw.hpp"
#include "ekfPos.hpp"

void gyroCallback(const fmMsgs::gyroscope::ConstPtr&);
void accCallback(const fmMsgs::accelerometer::ConstPtr&);
void magCallback(const fmMsgs::magnetometer::ConstPtr&);
void gpsCallback(const fmMsgs::gps_state::ConstPtr&);
void altCallback(const fmMsgs::altitude::ConstPtr&);
void pitotCallback(const fmMsgs::airSpeed::ConstPtr&);
void pubCallback(const ros::TimerEvent&);

ros::Publisher state_pub;
fmMsgs::airframeState state;

double wx = 0, wy = 0, wz = 0;
double temperature = 25;
double pressure = 1013.25 * 100;
double levelLimit = 0;

ekfAtt* attEstPtr;
ekfYaw* yawEstPtr;
ekfPos* posEstPtr;

int main(int argc, char** argv) {
	ros::init(argc, argv, "flyStixKalman");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	double accVar, gyroVar, magVar, gpsVar;
	n.param<double> ("accVar", accVar, 1);
	n.param<double> ("gyroVar", gyroVar, 1);
	n.param<double> ("magVar", magVar, 1);
	n.param<double> ("gpsVar", gpsVar, 1);
	n.param<double> ("isLevelThreshold", levelLimit, 15);
	levelLimit = levelLimit * M_PI / 180;

	/* INIT EXTENDED KALMAN FILTER FOR PITCH / ROLL*/
	ROS_INFO("fmEstimator : Initializing pitch/roll ekf...");
	const unsigned n_pr = 2;	//nb states
	const unsigned m_pr = 3;	//nb measures
	static const double _P0_pr[] = {10.0,  0.0,
								     0.0, 10.0};
	ekfAtt::Vector x_pr(n_pr);
	ekfAtt::Matrix P0_pr(n_pr, n_pr, _P0_pr);
	ekfAtt::Vector z_pr(m_pr);
	z_pr(1) = 0;
	z_pr(2) = 0;
	z_pr(3) = 0;
	ekfAtt::Vector u_pr(m_pr);
	x_pr(1) = 0;
	x_pr(2) = 0;

	ekfAtt attitudeEstimator(gyroVar, accVar);
	attitudeEstimator.init(x_pr, P0_pr);
	attEstPtr = &attitudeEstimator;
	ROS_INFO("fmEstimator : Done");

	/* INIT EXTENDED KALMAN FILTER FOR YAW */
	ROS_INFO("fmEstimator : Initializing yaw ekf...");

	static const double _P0_y[] = {10.0};
	ekfYaw::Vector x_y(1);
	ekfYaw::Matrix P0_y(1, 1, _P0_y);
	ekfYaw::Vector z_y(3);
	z_y(1) = 0;
	z_y(2) = 0;
	z_y(3) = 0;
	ekfAtt::Vector u_y(4);
	x_y(1) = 0;

	ekfYaw yawEstimator(gyroVar, magVar);
	yawEstimator.init(x_y, P0_y);
	yawEstPtr = &yawEstimator;
	ROS_INFO("fmEstimator : Done");


	/* INIT EXTENDED KALMAN FILTER FOR YAW */
	ROS_INFO("fmEstimator : Initializing position ekf...");

	static const double _P0_p[] = {1000.0,    0.0,    0.0,    0.0,    0.0,
	                                  0.0, 1000.0,    0.0,    0.0,    0.0,
	                                  0.0,    0.0, 1000.0,    0.0,    0.0,
	                                  0.0,    0.0,    0.0, 1000.0,    0.0,
	                                  0.0,    0.0,    0.0,    0.0, 1000.0};
	ekfPos::Vector x_p(5);
	ekfPos::Matrix P0_p(5, 5, _P0_p);
	ekfPos::Vector z_p(3);
	z_y(1) = 0;
	z_y(2) = 0;
	z_y(3) = 0;
	ekfAtt::Vector u_p(1);

	// FIXME : Wait for GPS fix... Some how?
	x_p(1) = 6148222.02;
	x_p(2) =  589484.64;
	x_p(3) = 0;
	x_p(4) = 0;
	x_p(5) = 0;

	ekfPos posEst(gyroVar, gpsVar);
	posEst.init(x_p, P0_p);
	posEstPtr = &posEst;

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
	ekfAtt::Vector uAtt(3);
	ekfAtt::Vector xAtt(2);
	ekfYaw::Vector uYaw(2);
	ekfYaw::Vector xYaw(1);
	ekfPos::Vector uPos(1);
	ekfPos::Vector xPos(5);

	wx = msg->vector.x;
	wy = msg->vector.y;
	wz = msg->vector.z;

	/* Do attitude timeUpdate : */
	uAtt(1) = wx * dt;
	uAtt(2) = wy * dt;
	uAtt(3) = wz * dt;

	attEstPtr->timeUpdateStep(uAtt);
	attEstPtr->updateAngVel(wx,wy,wz);
	xAtt = attEstPtr->getX();

	/* Do yaw timeUpdate : */
	uYaw(1) = wy * dt;
	uYaw(2) = wz * dt;

	yawEstPtr->updateAttitude(state.pose.x, state.pose.y);
	yawEstPtr->timeUpdateStep(uYaw);
	xYaw = yawEstPtr->getX();

	/* Do position timeUpdate */
	uPos(1) = dt;
	ekfPos::Vector xB = posEstPtr->getX();

	posEstPtr->updatePose(state.pose.x, state.pose.y, state.pose.z);
	posEstPtr->timeUpdateStep(uPos);

	ekfPos::Vector xA = posEstPtr->getX();

//	printf("dt = %2.6f\n", uPos(1));
//	printf("X = [%2.6f , %2.6f , %2.6f , %2.6f , %2.6f]\n",
//	       xA(1)-xB(1),
//	       xA(2)-xB(2),
//	       xA(3)-xB(3),
//	       xA(4)-xB(4),
//	       xA(5)-xB(5));

	xPos = posEstPtr->getX();

	/* Save back to airframe state */
	state.pose.x = xAtt(1);
	state.pose.y = xAtt(2);
	state.pose.z = xYaw(1);
	state.Pn = xPos(1);
	state.Pe = xPos(2);
	state.Wn = xPos(3);
	state.We = xPos(4);
	state.airspeed = xPos(5);
}

void accCallback(const fmMsgs::accelerometer::ConstPtr& msg) {
	ekfAtt::Vector z(3);
	ekfAtt::Vector x(2);
	z(1) = msg->vector.x;
	z(2) = msg->vector.y;
	z(3) = msg->vector.z;

	double v = asin(msg->vector.y / sqrt(pow(msg->vector.y,2) + pow(msg->vector.z,2)));
	state.incline = v * 0.1 + state.incline * 0.9;

	attEstPtr->measureUpdateStep(z);
	x = attEstPtr->getX();

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
	yawEstPtr->updateAttitude(state.pose.x, state.pose.y);
	yawEstPtr->measureUpdateStep(z);
	x = yawEstPtr->getX();
	/* Update state */
	state.pose.z = x(1);
}

void gpsCallback(const fmMsgs::gps_state::ConstPtr& msg) {
	ekfPos::Vector z(3);
	ekfPos::Vector x(5);
	static double _GPSn = 0;
	static double _GPSe = 0;
	static double _t = 0;

	bool doBreak = (_GPSn == 0 || _GPSe == 0 || _t == 0);
	if (msg->fix < 1)
		return;

	double GPSn = msg->utm_n;
	double GPSe = msg->utm_e;
	double t = msg->header.stamp.toSec();

	if (doBreak) {
		_GPSn = GPSn;
		_GPSe = GPSe;
		_t = t;
		ROS_DEBUG("Skipping init GPS sample");
		return;
	}

	double Vg = sqrt(pow(GPSn - _GPSn, 2) + pow(GPSe - _GPSe, 2)) / (t - _t);

	_GPSn = GPSn;
	_GPSe = GPSe;
	_t = t;

	z(1) = GPSn;
	z(2) = GPSe;
	z(3) = Vg;

	posEstPtr->updatePose(state.pose.x, state.pose.y, state.pose.z);
	posEstPtr->measureUpdateStep(z);
	x = posEstPtr->getX();
	/* Update state */
	state.Pn = x(1);
	state.Pe = x(2);
	state.Wn = x(3);
	state.We = x(4);
	state.airspeed = x(5);

	ekfPos::Vector zP = posEstPtr->simulate();
	printf("State    [%2.2f , %2.2f , %2.2f , %2.2f , %2.2f]\n"
		   "Expected [%2.2f , %2.2f , %2.2f]\n"
		   "Got      [%2.2f , %2.2f , %2.2f]\n\n",
		    x(1),x(2),x(3),x(4),x(5),zP(1),zP(2),zP(3),z(1),z(2),z(3));

    state.lat = msg->lat;
    state.lon = msg->lon;
}

void altCallback(const fmMsgs::altitude::ConstPtr& msg) {
	temperature = msg->temperature;
	pressure = msg->pressure;
	state.alt = msg->altitude;
	state.dist = msg->range;
}

void pitotCallback(const fmMsgs::airSpeed::ConstPtr& msg) {
	/* Update vAir */
	/* 	msg->airspeed;
	 * msg->stamp; */
	/* Update state */
//	state.airspeed = msg->airspeed;
	attEstPtr->updateAirspeed(1);
	double Vp, we, wn, ph, th, ps, alfa, beta, ga;
	wn = state.Wn;
	we = state.We;
	ph = state.pose.x;
	th = state.pose.y;
	ps = state.pose.z;

	/*
	 * msg->airspeed = 3.3V / 1023 * Vamp
	 * Vamp = Vmpxv * 0.8 - 2.5V
	 * Vmpxv = Pdiff * 1mV/Pa + 2.5V
	 * Pdiff = 1/2 * rho * Va^2
	 *     rho = (P[hPa] * 10^2) / 287.058 * 1 / (T + 273.15)
	 *         P = Pressure [hPa]
	 *         T = Temperature [C]
	 * => Va = sqrt(2 * 3.3V/1023LSB * 5/4 * msg->airspeed / rho)
	 *       = sqrt(8.064516129 * msg->airspeed / rho)
	 */

	double rho = (pressure / 287.085) * (1 / (temperature + 273.15));
	Vp = sqrt(8.064516129 * msg->airspeed / rho);

	alfa = 0;	// Angle of attack
	beta = 0;	// Slip angle
	ga = th - alfa*cos(ph) - beta*sin(ph); // Inertial climb angle
	state.airspeed = sqrt(pow((Vp*cos(ps)*cos(ga) - wn),2) +
	                      pow((Vp*sin(ps)*cos(ga) - we),2) +
	                      pow((Vp*sin(ga)),2));
}




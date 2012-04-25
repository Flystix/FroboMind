#include "kalman.hpp"

#include <ros/ros.h>
#include <fmMsgs/fmVector3.h>

ros::Publisher simPub;

enum {
    PITOT,
    BAROMETER,
    GPS
};
char* meterName[] = {
		"Pitot",
		"Barometer",
		"GPS"
};

int speedometer = GPS;
int altimeter = GPS;

kalman::kalman(ros::NodeHandle& nh, ros::NodeHandle& n) {
	sem_init(&attEstLock, 0, 1);
	sem_init(&yawEstLock, 0, 1);
	sem_init(&posEstLock, 0, 1);

	simPub = nh.advertise<fmMsgs::fmVector3>("/simData", 1, 0);

	wx = 0;
	wy = 0;
	wz = 0;
	temperature = 25;
	pressure = 1013.25;
	Alt = 0;

	state.Pe = 0;
	state.Pn = 0;
	state.We = 0;
	state.Wn = 0;
	state.Va = 0;
	state.Vi = 0;
	state.alt = 0;
	state.climb = 0;
	state.dist = 0;
	state.header.frame_id = "AirFrame";
	state.header.seq = 0;
	state.header.stamp = ros::Time::now();
	state.incline = 0;
	state.lat = 0;
	state.lon = 0;
	state.pose.x = 0;
	state.pose.y = 0;
	state.pose.z = 0;

	double accVar, gyroVar, magVar, gpsVar, pitotVar, pubRate, altVar;
	n.param<double> ("accVar", accVar, 1);
	n.param<double> ("gyroVar", gyroVar, 1);
	n.param<double> ("magVar", magVar, 1);
	n.param<double> ("gpsVar", gpsVar, 10);
	n.param<double> ("altVar", altVar, 10);
	n.param<double> ("pitotVar", pitotVar, 1);
	n.param<double> ("pubRate", pubRate, 50);

	std::string speedometerstr;
	n.param<std::string> ("speedometer", speedometerstr, meterName[GPS]);
	if (!speedometerstr.compare(meterName[GPS]))
		speedometer = GPS;
	else if (!speedometerstr.compare(meterName[PITOT]))
		speedometer = PITOT;
	else
		speedometer = GPS;
	std::string altimeterstr;
	n.param<std::string> ("altimeter", altimeterstr, meterName[GPS]);
	if (!altimeterstr.compare(meterName[GPS]))
		altimeter= GPS;
	else if (!altimeterstr.compare(meterName[BAROMETER]))
		altimeter= BAROMETER;
	else
		altimeter= GPS;

	ROS_INFO("kalman : gyroscope variance : %2.2f", gyroVar);
	ROS_INFO("kalman : accelerometer variance : %2.2f", accVar);
	ROS_INFO("kalman : magnetometer variance : %2.2f", magVar);
	ROS_INFO("kalman : GPS variance : %2.2f", gpsVar);
	ROS_INFO("kalman : Speedometer instrument : %s", meterName[speedometer]);
	ROS_INFO("kalman : Altimeter instrument : %s", meterName[altimeter]);
	ROS_INFO("kalman : Altimeter variance: %2.2f", altVar);

	/* INIT EXTENDED KALMAN FILTER FOR PITCH / ROLL*/
	ROS_INFO("fmFusion : Initializing pitch/roll ekf...");
	attitudeEstimator = new ekfAttQuat(gyroVar, accVar);
	attitudeEstimator->updateAirspeed(state.Va);
	attitudeEstimator->reset(0, 0);
	ekfAttQuat::Vector xAttInit = attitudeEstimator->getXEuler();
	ROS_INFO("Initial state : x = [%2.2f, %2.2f]", xAttInit(1), xAttInit(2));

	/* INIT EXTENDED KALMAN FILTER FOR YAW */
	ROS_INFO("fmFusion : Initializing yaw ekf...");
	headingEstimator = new ekfYaw(gyroVar, magVar, n);
	headingEstimator->reset(0);
	ekfYaw::Vector xYawInit = headingEstimator->getX();
	ROS_INFO("Initial state : x = [%2.2f]", xYawInit(1));

	/* INIT EXTENDED KALMAN FILTER FOR POSITON/WIND */
	ROS_INFO("fmFusion : Initializing position ekf...");
	positionEstimator = new ekfPos(gyroVar, gpsVar, altVar);
	positionEstimator->reset(0,0,0,0,1.2250);
	ekfPos::Vector xPosInit = positionEstimator->getX();
	ROS_INFO("Initial state : x = [%2.2f , %2.2f , %2.2f , %2.2f]", xPosInit(1), xPosInit(2), xPosInit(3), xPosInit(4));

	ROS_INFO("fmFusion : Done");

	state.header.frame_id = "AirFrame";
	state.header.seq = 0;
	state_pub = nh.advertise<fmMsgs::airframeState> ("/airframeState", 1);
}

kalman::~kalman() {
	delete attitudeEstimator;
	delete headingEstimator;
	//	delete positionEstimator;
}

void kalman::gyroCallback(const fmMsgs::gyroscope& msg) {
	static ros::Time _stamp = ros::Time::now();
	ros::Time stamp = msg.stamp;
	stamp = ros::Time::now();
	double dt = (stamp - _stamp).toSec();
	_stamp = stamp;
	ekfAttQuat::Vector uAtt(3);
	ekfAttQuat::Vector xAtt(2);
	ekfYaw::Vector uYaw(2);
	ekfYaw::Vector xYaw(1);
	ekfPos::Vector uPos(4);
	ekfPos::Vector xPos(8);

	wx = msg.vector.x;
	wy = msg.vector.y;
	wz = msg.vector.z;

	/* Do attitude timeUpdate : */
	uAtt(1) = wx * dt;
	uAtt(2) = wy * dt;
	uAtt(3) = wz * dt;

	sem_wait(&attEstLock);
	attitudeEstimator->updatedt(dt);
	attitudeEstimator->updateAngVel(wx, wy, wz);
	attitudeEstimator->timeUpdateStep(uAtt);
	sem_post(&attEstLock);
	xAtt = attitudeEstimator->getXEuler();
	/* Save back to airframe state */
	state.pose.x = xAtt(1);
	state.pose.y = xAtt(2);

	/* Do yaw timeUpdate : */
	uYaw(1) = wy * dt;
	uYaw(2) = wz * dt;

	sem_wait(&yawEstLock);
	headingEstimator->updateAttitude(state.pose.x, state.pose.y);
	headingEstimator->timeUpdateStep(uYaw);
	sem_post(&yawEstLock);
	xYaw = headingEstimator->getX();
	/* Save back to airframe state */
	state.pose.z = xYaw(1);

	/* Do Position timeUpdate u = [th ; ps; Pd ; dt];*/
	uPos(1) = state.pose.y;
	uPos(2) = state.pose.z;
	uPos(3) = Pd;
	uPos(4) = dt;

	sem_wait(&posEstLock);
	positionEstimator->timeUpdateStep(uPos);
	xPos = positionEstimator->getX();
	sem_post(&posEstLock);

	//	state.Va = state.Vg / cos(state.pose.y);
	//	attitudeEstimator->updateAirspeed(state.Va);
	double Wn = xPos(3);
	double We = xPos(4);

	if (speedometer == PITOT) {
		state.Va = sqrt(pow(state.Vi*cos(state.pose.y)*cos(state.pose.z) - Wn, 2) +
		                pow(state.Vi*cos(state.pose.y)*sin(state.pose.z) - We, 2) +
		                pow(state.Vi*sin(state.pose.y), 2)) * 0.025 + state.Va * 0.975;
		attitudeEstimator->updateAirspeed(state.Va);
	}

	/* Save back to airframe state */
	state.Pn = xPos(1);
	state.Pe = xPos(2);
	state.Wn = xPos(3);
	state.We = xPos(4);
	state.alt = xPos(7);
	state.climb = xPos(8);

	state.header.stamp = ros::Time::now();
}

void kalman::accCallback(const fmMsgs::accelerometer& msg) {
	ekfAttQuat::Vector z(3);
	z(1) = msg.vector.x;
	z(2) = msg.vector.y;
	z(3) = msg.vector.z;

	double v = asin(msg.vector.y / sqrt(pow(msg.vector.y, 2) + pow(msg.vector.z, 2)));
	state.incline = v * 0.1 + state.incline * 0.9;

	sem_wait(&attEstLock);
	attitudeEstimator->measureUpdateStep(z);
	sem_post(&attEstLock);
}

void kalman::magCallback(const fmMsgs::magnetometer& msg) {
	// Run measure-update on stage 2
	ekfYaw::Vector z(3);

	/* ALTERNATIVE (And more politically correct):
	 * Manipulate magnetometer confidence / covariance
	 * according to of-levelness...
	 */
	z(1) = msg.vector.x;
	z(2) = msg.vector.y;
	z(3) = msg.vector.z;
	sem_wait(&yawEstLock);
	headingEstimator->updateAttitude(state.pose.x, state.pose.y);
	headingEstimator->measureUpdateStep(z);
	sem_post(&yawEstLock);
}

void kalman::gpsCallback(const fmMsgs::gps_state& msg) {
	static bool gotFix = 0;
	ekfPos::Vector z(3);

	if (!gotFix && msg.fix) {
		positionEstimator->reset(msg.utm_n, msg.utm_e, 0.0, 0.0, 1.288723668);
		ROS_INFO("Resetting to %2.2fN , %2.2fE", msg.utm_n, msg.utm_e);
	}

	gotFix = (bool) msg.fix;

	if (!gotFix)
		return;

	state.lat = msg.lat;
	state.lon = msg.lon;

	z(1) = msg.utm_n;
	z(2) = msg.utm_e;
	if (altimeter == BAROMETER)
		z(3) = Alt;
	else
		z(3) = msg.alt;


//	ekfPos::Matrix P(8,8);
//	P = positionEstimator->calculateP();
//		ROS_INFO("P = \t[%2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f]", P(1,1), P(1,2), P(1,3), P(1,4), P(1,5), P(1,6), P(1,7), P(1,8));
//		ROS_INFO("    \t[%2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f]", P(2,1), P(2,2), P(2,3), P(2,4), P(2,5), P(2,6), P(2,7), P(2,8));
//		ROS_INFO("    \t[%2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f]", P(3,1), P(3,2), P(3,3), P(3,4), P(3,5), P(3,6), P(3,7), P(3,8));
//		ROS_INFO("    \t[%2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f]", P(4,1), P(4,2), P(4,3), P(4,4), P(4,5), P(4,6), P(4,7), P(4,8));
//		ROS_INFO("    \t[%2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f]", P(5,1), P(5,2), P(5,3), P(5,4), P(5,5), P(5,6), P(5,7), P(5,8));
//		ROS_INFO("    \t[%2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f]", P(6,1), P(6,2), P(6,3), P(6,4), P(6,5), P(6,6), P(6,7), P(6,8));
//		ROS_INFO("    \t[%2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f]", P(7,1), P(7,2), P(7,3), P(7,4), P(7,5), P(7,6), P(7,7), P(7,8));
//		ROS_INFO("    \t[%2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f , %2.6f]", P(8,1), P(8,2), P(8,3), P(8,4), P(8,5), P(8,6), P(8,7), P(8,8));

	sem_wait(&posEstLock);
	positionEstimator->measureUpdateStep(z);
	sem_post(&posEstLock);

	state.Vg = msg.kph / 3.6;
	if (speedometer == GPS) {
		state.Va = state.Vg / cos(state.pose.y);
		attitudeEstimator->updateAirspeed(state.Va);
	}
}

void kalman::altCallback(const fmMsgs::altitude& msg) {
	temperature = msg.temperature;
	pressure = msg.pressure; // Pressure in hPa
	Alt = msg.altitude;
	state.dist = msg.range;
}

void kalman::pitotCallback(const fmMsgs::airSpeed& msg) {
	Pd = msg.airspeed * 0.5 + Pd * 0.5;
	state.Vi = positionEstimator->updateVi(msg.airspeed); // Indicated airspeed
}

fmMsgs::airframeState* kalman::getState(void) {
	return &state;
}

#include "kalman.hpp"

kalman::kalman(ros::NodeHandle& nh, ros::NodeHandle& n) {
	sem_init(&attEstLock, 0, 1);
	sem_init(&yawEstLock, 0, 1);
	sem_init(&posEstLock, 0, 1);

	wx = 0;
	wy = 0;
	wz = 0;
	temperature = 25;
	pressure = 1013.25;

	state.Wn = 0;
	state.We = 0;

	double accVar, gyroVar, magVar, gpsVar, pitotVar, pubRate;
	n.param<double>("accVar", accVar, 1);
	n.param<double>("gyroVar", gyroVar, 1);
	n.param<double>("magVar", magVar, 1);
	n.param<double>("gpsVar", gpsVar, 10);
	n.param<double>("pitotVar", pitotVar, 1);
	n.param<double>("pubRate", pubRate, 50);
	n.param<double>("pitotOffset", pitotOffset, 14.8);
	ROS_INFO("kalman : gyroscope variance : %2.2f", gyroVar);
	ROS_INFO("kalman : accelerometer variance : %2.2f", accVar);
	ROS_INFO("kalman : magnetometer variance : %2.2f", magVar);
	ROS_INFO("kalman : GPS variance : %2.2f", gpsVar);
	ROS_INFO("kalman : Pitot variance : %2.2f", pitotVar);

	/* INIT EXTENDED KALMAN FILTER FOR PITCH / ROLL*/
	ROS_INFO("fmEstimator : Initializing pitch/roll ekf...");
	attitudeEstimator = new ekfAttQuat(gyroVar, accVar);
	resetAtt(0,0);
	ROS_INFO("fmEstimator : Done");

	/* INIT EXTENDED KALMAN FILTER FOR YAW */
	ROS_INFO("fmEstimator : Initializing yaw ekf...");
	headingEstimator = new ekfYaw(gyroVar, magVar, n);
	resetYaw(0);
	ROS_INFO("fmEstimator : Done");

//	ROS_INFO("fmEstimator : Initializing position ekf....");
//	static const double _P0_p[] = {10000000.0, 1.0, 1.0, 1.0,
//								   1.0, 10000000.0, 1.0, 1.0,
//								   1.0, 1.0, 10000000.0, 1.0,
//							 	   1.0, 1.0, 1.0, 10000000.0};
//	ekfPos::Vector x_p(4);
//	ekfPos::Matrix P0_p(4, 4, _P0_p);
//	ekfPos::Vector z_p(2);
//	z_p(1) = 0;
//	z_p(2) = 0;
//	z_p(3) = 0;
//	ekfPos::Vector u_p()
//	ROS_INFO("fmEstimator : Done");

	state.header.frame_id = "AirFrame";
	state.header.seq = 0;
	state_pub = nh.advertise<fmMsgs::airframeState>("/airframeState", 1);

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

	wx = msg.vector.x;
	wy = msg.vector.y;
	wz = msg.vector.z;

	/* Do attitude timeUpdate : */
	uAtt(1) = wx * dt;
	uAtt(2) = wy * dt;
	uAtt(3) = wz * dt;

	attitudeEstimator->updateAngVel(wx, wy, wz);

	sem_wait(&attEstLock);
	attitudeEstimator->timeUpdateStep(uAtt);
	sem_post(&attEstLock);
	xAtt = attitudeEstimator->getX();

	/* Do yaw timeUpdate : */
	uYaw(1) = wy * dt;
	uYaw(2) = wz * dt;

	sem_wait(&yawEstLock);
	headingEstimator->updateAttitude(state.pose.x, state.pose.y);
	headingEstimator->timeUpdateStep(uYaw);
	sem_post(&yawEstLock);
	xYaw = headingEstimator->getX();

	/* Save back to airframe state */
	ROS_DEBUG("GYRO : x = : %2.2f, %2.2f\n", xAtt(1), xAtt(2));
	state.pose.x = xAtt(1);
	state.pose.y = xAtt(2);
	state.pose.z = xYaw(1);
}

void kalman::accCallback(const fmMsgs::accelerometer& msg) {
	ekfAttQuat::Vector z(3);
	ekfAttQuat::Vector x(2);
	z(1) = msg.vector.x;
	z(2) = msg.vector.y;
	z(3) = msg.vector.z;

	double v = asin(
			msg.vector.y / sqrt(pow(msg.vector.y, 2) + pow(msg.vector.z, 2)));
	state.incline = v * 0.1 + state.incline * 0.9;

	sem_wait(&attEstLock);
	attitudeEstimator->measureUpdateStep(z);
	sem_post(&attEstLock);
	x = attitudeEstimator->getX();

	state.pose.x = x(1);
	state.pose.y = x(2);
}

void kalman::magCallback(const fmMsgs::magnetometer& msg) {
	// Run measure-update on stage 2
	ekfYaw::Vector z(3);
	ekfYaw::Vector x(1);

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
	x = headingEstimator->getX();
	/* Update state */
	state.pose.z = x(1);
}

void kalman::gpsCallback(const fmMsgs::gps_state& msg) {
	state.lat = msg.lat;
	state.lon = msg.lon;
}

void kalman::altCallback(const fmMsgs::altitude& msg) {
	temperature = msg.temperature;
	pressure = msg.pressure; // Pressure in hPa
	state.alt = msg.altitude;
	state.dist = msg.range;
}

void kalman::pitotCallback(const fmMsgs::airSpeed& msg) {
	double Vp, Vi, we, wn, ph, th, ps, alfa, beta, ga;
	static int i = 0;
	wn = state.Wn;
	we = state.We;
	ph = state.pose.x;
	th = state.pose.y;
	ps = state.pose.z;

	double rho = (pressure * 100 / 287.085) * (1 / (temperature + 273.15));
	Vp = (msg.airspeed < pitotOffset ? 0 : msg.airspeed - pitotOffset); // Pitot data
	Vi = sqrt(8.064516129 * Vp / rho); // Indicated airspeed

	alfa = 0; // Angle of attack
	beta = 0; // Slip angle
	ga = th - alfa * cos(ph) - beta * sin(ph); // Inertial climb angle
	state.airspeed = sqrt(
			pow((Vi * cos(ps) * cos(ga) - wn), 2)
					+ pow((Vi * sin(ps) * cos(ga) - we), 2)
					+ pow((Vi * sin(ga)), 2)); // True airspeed

	state.airspeed = msg.airspeed;
	attitudeEstimator->updateAirspeed(state.airspeed);
}

fmMsgs::airframeState* kalman::getState(void) {
	return &state;
}

//void kalman::resetAtt(double initPhi, double initTheta) {
//	double P0[] = { 2 * M_PI, 0.0, 0.0, 2 * M_PI };
//	double x0[] = { initPhi, initTheta };
//	ekfAttQuat::Vector x(2, x0);
//	ekfAttQuat::Matrix P(2, 2, P0);
//	attitudeEstimator->init(x, P);
//}

void kalman::resetAtt(double initPhi, double initTheta) {
	double P0[] = { 1.0, 0.0, 0.0, 0.0,
	                0.0, 1.0, 0.0, 0.0,
	                0.0, 0.0, 1.0, 0.0,
	                0.0, 0.0, 0.0, 1.0};
	double ph = initPhi / 2;
	double th = initTheta / 2;
	double ps = 0;
	double x0[] = {
		cos(ph)*cos(th)*cos(ps) + sin(ph)*sin(th)*sin(ps),
		sin(ph)*cos(th)*cos(ps) - cos(ph)*sin(th)*sin(ps),
		cos(ph)*sin(th)*cos(ps) + sin(ph)*cos(th)*sin(ps),
		cos(ph)*cos(th)*sin(ps) - sin(ph)*sin(th)*cos(ps)
	};

	ekfAttQuat::Vector x1(4, x0);
	ekfAttQuat::Matrix P(4, 4, P0);
	attitudeEstimator->init(x1, P);
}

void kalman::resetYaw(double initPsi) {
	double P0[] = { 2 * M_PI };
	double x0[] = { initPsi };
	ekfYaw::Vector x_pr(1, x0);
	ekfYaw::Matrix P0_pr(1, 1, P0);
	headingEstimator->init(x_pr, P0_pr);
}
void kalman::resetPos(double initPn, double initPe) {
	const double var = 1000.0;
	double P0[] = { var, 0.0, 0.0, 0.0,
				    0.0, var, 0.0, 0,0,
	 	 	 	 	0.0, 0.0, var, 0.0,
				    0.0, 0.0, 0.0, var};
	double x0[] = { initPn, initPe, 0.0, 0.0 };
//	ekfPos::Vector x(4, x0);
//	ekfPos::Matrix P(4, 4, P0);
//	positionEstimator->init(x, P);
}

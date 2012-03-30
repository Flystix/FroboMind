#include "kalman.hpp"

kalman::kalman(ros::NodeHandle& nh, ros::NodeHandle& n) {
	sem_init(&attEstLock, 0, 1);
	sem_init(&yawEstLock, 0, 1);

	wx = 0;
	wy = 0;
	wz = 0;
	temperature = 25;
	pressure = 1013.25;
	levelLimit = 0;

	state.Wn = 0;
	state.We = 0;

	double accVar, gyroVar, magVar, pubRate;
	n.param<double> ("accVar", accVar, 1);
	n.param<double> ("gyroVar", gyroVar, 1);
	n.param<double> ("magVar", magVar, 1);
	n.param<double> ("pubRate", pubRate, 50);
	n.param<double> ("isLevelThreshold", levelLimit, 15);
	n.param<double> ("pitotOffset", pitotOffset, 14.8);
	levelLimit = levelLimit * M_PI / 180;

	/* INIT EXTENDED KALMAN FILTER FOR PITCH / ROLL*/
	ROS_INFO("fmEstimator : Initializing pitch/roll ekf...");
	const unsigned n_pr = 2;	//nb states
	const unsigned m_pr = 3;	//nb measures
	static const double _P0_pr[] = {1.0, 0.0, 0.0, 1.0};
	ekfAtt::Vector x_pr(n_pr);
	ekfAtt::Matrix P0_pr(n_pr, n_pr, _P0_pr);
	ekfAtt::Vector z_pr(m_pr);
	z_pr(1) = 0;
	z_pr(2) = 0;
	z_pr(3) = 0;
	ekfAtt::Vector u_pr(m_pr);
	x_pr(1) = 0;
	x_pr(2) = 0;

	attitudeEstimator = new ekfAtt(gyroVar, accVar);
	attitudeEstimator->init(x_pr, P0_pr);
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
	ekfAtt::Vector u_y(4);
	x_y(1) = 0;

	headingEstimator = new ekfYaw(gyroVar, magVar);
	headingEstimator->init(x_y, P0_y);
	ROS_INFO("fmEstimator : Done");

	state.header.frame_id = "AirFrame";
	state.header.seq = 0;
//	ROS_INFO("fmEstimator : Subscribing to topics");
//	ros::Subscriber gyro_sub = nh.subscribe("/gyroData", 1, gyroCallback);
//	ros::Subscriber acc_sub  = nh.subscribe("/accData", 1, accCallback);
//	ros::Subscriber mag_sub  = nh.subscribe("/magData", 1, magCallback);
//	ros::Subscriber gps_sub  = nh.subscribe("/fmExtractors/gps_state_msg", 1, gpsCallback);
//	ros::Subscriber alt_sub  = nh.subscribe("/altData", 1, altCallback);
//	ros::Subscriber pit_sub  = nh.subscribe("/pitotData", 1, pitotCallback);

	state_pub = nh.advertise<fmMsgs::airframeState>("/airframeState", 1);
	pub_timer = nh.createTimer(ros::Duration(1/pubRate), &kalman::pubCallback, this);
	//ROS_INFO("fmEstimator : Spinning...");
	//ros::spin();
}

kalman::~kalman() {
	delete attitudeEstimator;
	delete headingEstimator;
}

void kalman::pubCallback(const ros::TimerEvent& e) {
	/* publish state */
	state.header.stamp = ros::Time::now();
	state.header.seq++;
	state_pub.publish(state);
}

void kalman::gyroCallback(const fmMsgs::gyroscope& msg) {
	static ros::Time _stamp = ros::Time::now();
	double dt = (msg.stamp - _stamp).toSec();
	_stamp = msg.stamp;
	ekfAtt::Vector uAtt(3);
	ekfAtt::Vector xAtt(2);
	ekfYaw::Vector uYaw(2);
	ekfYaw::Vector xYaw(1);

	wx = msg.vector.x;
	wy = msg.vector.y;
	wz = msg.vector.z;

	/* Do attitude timeUpdate : */
	uAtt(1) = wx * dt;
	uAtt(2) = wy * dt;
	uAtt(3) = wz * dt;

	attitudeEstimator->updateAngVel(wx,wy,wz);

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
	ekfAtt::Vector z(3);
	ekfAtt::Vector x(2);
	z(1) = msg.vector.x;
	z(2) = msg.vector.y;
	z(3) = msg.vector.z;

	double v = asin(msg.vector.y / sqrt(pow(msg.vector.y,2) + pow(msg.vector.z,2)));
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

	/* Discard magnetometer data, if flight isn't some-what level*/
	if(state.pose.x > levelLimit || state.pose.x < -levelLimit ||
       state.pose.y > levelLimit || state.pose.y < -levelLimit)
		return;

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

	alfa = 0;	// Angle of attack
	beta = 0;	// Slip angle
	ga = th - alfa*cos(ph) - beta*sin(ph); // Inertial climb angle
	state.airspeed = sqrt(pow((Vi*cos(ps)*cos(ga) - wn),2) +
	                      pow((Vi*sin(ps)*cos(ga) - we),2) +
	                      pow((Vi*sin(ga)),2)); // True airspeed
//	if (i++ % 10 == 0) {
//		char buf[128];
//		sprintf(buf, "\n");
//		ROS_WARN("%s", buf);
//		sprintf(buf, "Pressure :       %2.2f[hPa]", pressure);
//		ROS_WARN(buf, "%s", buf);
//		sprintf(buf, "Temperature :    %2.2f[C]", temperature);
//		ROS_WARN(buf, "%s", buf);
//		sprintf(buf, "Rho :            %2.2f[kg/m^3]", rho);
//		ROS_WARN("%s", buf);
//		sprintf(buf, "Airspeed[raw]:   %2.2f[?]", msg.airspeed);
//		ROS_WARN("%s", buf);
//		sprintf(buf, "Airspeed[ind] :  %2.2f[m/s]", Vi);
//		ROS_WARN("%s", buf);
//		sprintf(buf, "Airspeed[true] : %2.2f[m/s]", state.airspeed);
//		ROS_WARN("%s", buf);
//	}

	attitudeEstimator->updateAirspeed(state.airspeed);
}

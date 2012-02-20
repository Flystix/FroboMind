#include <ros/ros.h>
#include <fmMsgs/teleAir2Ground.h>	// To be transmitted via XBee
#include <fmMsgs/battery.h>			// For voltage / State of Charge
#include <fmMsgs/gps_state.h>		// For fix and nSatelites
#include <fmMsgs/altitude.h>		// For ultrasonic and barometric altitude
#include <fmMsgs/airSpeed.h>		// For uncompendsated airspeed
// #include <fmMsgs/pose.h>			// For 6DoF position and 2DoF wind estimates (output from Kalman)

fmMsgs::teleAir2Ground air2gnd;

void batteryCallback(const fmMsgs::battery&);
void gpsCallback(const fmMsgs::gps_state&);
void altCallback(const fmMsgs::altitude&);
void speedCallback(const fmMsgs::airSpeed&);
void serialCallback(const fmMsgs::serial&);

int main(int argc, char** argv) {
	ros::init(argc,argv,"fmTeleNode");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	ros::Subscriber bat_sub = nh.subscribe<fmMsgs::battery>("/batteryData", 1, batteryCallback);
	ros::Subscriber gps_sub = nh.subscribe<fmMsgs::gps_state>("/gps_state", 1, gpsCallback);
	ros::Subscriber alt_sub = nh.subscribe<fmMsgs::altitude>("/altData", 1, altCallback);
	ros::Subscriber spe_sub = nh.subscribe<fmMsgs::airSpeed>("/speedData", 1, speedCallback);
	ros::Subscriber ser_sub = nh.subscribe<fmMsgs::serial>("/tele_rx", 1, serialCallback);
	ros::Publisher  ser_pub = nh.advertise<gmMsgs::serial>("/tele_tx", 1);
	ros::Publisher  tel_pub = nh.advertise<gmMsgs::teleAir2Ground>("/teleData", 1);

	ros::Rate wait(10);

	while(ros::ok()) {
		// Serialize air2gnd
		// Wrap
		// SLIP
		// Publish using ser_pub
		wait.sleep();
	}
}

void serialCallback(const fmMsgs::serial& msg) {
	/* unSLIP */
	/* Unwrap */
	/* Deserialize */
	/* Publish using tel_pub */
}

void batteryCallback(const fmMsgs::battery& msg) {
	air2gnd.battery = msg.voltage;
}

void gpsCallback(const fmMsgs::gps_state& msg) {
	air2gnd.gps_fix = msg.fix;
	air2gnd.gps_sats = msg.sat;
}

void altCallback(const fmMsgs::altitude& msg) {
	air2gnd.position.z = msg.altitude;
}

void speedCallback(const fmMsgs::airSpeed& msg) {
	air2gnd.airspeed = msg.airspeed;
}

//void kalmanCallback(const fmMsgs::kalman& msg) {
//	air2gnd.position.x = msg.north;
//	air2gnd.position.y = msg.east;
//	air2gnd.orientation.x = msg.orientation.r;
//	air2gnd.orientation.y = msg.orientation.p;
//	air2gnd.orientation.z = msg.orientation.q;
//	air2gnd.truespeed = msg.truespeed;
//}

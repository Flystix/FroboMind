#include <ros/ros.h>
#include <fmMsgs/serial.h>
#include <fmMsgs/teleAir2Ground.h>	// To be transmitted via XBee
#include <fmMsgs/teleGround2Air.h>	// To be received via XBee
#include <fmMsgs/battery.h>			// For voltage / State of Charge
#include <fmMsgs/gps_state.h>		// For fix and nSatelites
#include <fmMsgs/altitude.h>		// For ultrasonic and barometric altitude
#include <fmMsgs/airSpeed.h>		// For uncompendsated airspeed
// #include <fmMsgs/pose.h>			// For 6DoF position and 2DoF wind estimates (output from Kalman)

#include "slip.hpp"

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

	ros::Subscriber bat_sub = nh.subscribe("/batteryData", 1, batteryCallback);
	ros::Subscriber gps_sub = nh.subscribe("/gps_state", 1, gpsCallback);
	ros::Subscriber alt_sub = nh.subscribe("/altData", 1, altCallback);
	ros::Subscriber spe_sub = nh.subscribe("/speedData", 1, speedCallback);
	ros::Subscriber ser_sub = nh.subscribe("/tele_rx", 1, serialCallback);
	ros::Publisher  ser_pub = nh.advertise<fmMsgs::serial>("/tele_tx", 1);
//	ros::Publisher  tel_pub = nh.advertise<fmMsgs::teleGround2Air>("/teleData", 1);

	ros::Rate rate(10);

	air2gnd.battery = 15.8;
	air2gnd.gps_fix = 3;
	air2gnd.gps_sats = 5;
	air2gnd.airspeed = 52.1;
	air2gnd.truespeed = 49.2;
	air2gnd.groundspeed = 40.23;
	air2gnd.position.x = 1212.23;
	air2gnd.position.y = 702.23;
	air2gnd.position.z = 20.12;
	air2gnd.orientation.x = 0.234;
	air2gnd.orientation.y = -2.13;
	air2gnd.orientation.z = 2.023;
	air2gnd.memutil = 25.52;
	air2gnd.cpuload = 12.24;
	air2gnd.header.seq = 0;
	air2gnd.header.frame_id = "AirFrame";

	uint32_t seq = 0;

	while(ros::ok()) {
		/* Update header */
		air2gnd.header.seq++;
		air2gnd.header.stamp = ros::Time::now();

		/* Serialize air2gnd */
		printf("Serializing...\n");
		uint32_t size = ros::serialization::serializationLength(air2gnd);
		uint32_t mallocSize = size + 11; // msgType(4) + msgSize(4) + size + chksum(2) + \n
		boost::shared_array<uint8_t> bufOut(new uint8_t[size]);
		ros::serialization::OStream streamOut(bufOut.get(), size);
		ros::serialization::serialize(streamOut, air2gnd);
		printf("\tsize : %i\n", size);

		/* Wrap */
		printf("Wrapping...\n");
		uint8_t* data = (uint8_t*)malloc(mallocSize);
		printf("\tAllocated %i bytes @ %08X...\n", mallocSize, (uint32_t)data);
		uint32_t len = 0;
		len += sprintf((char*)data, "%04X%04X", 0x1000, size); // MAX 16 bits each..!
		memcpy(data+8, bufOut.get(), size);
		len += size;
		uint8_t chksum = 0;
		for (uint32_t i = 0 ; i < size + 8 ; i++)
			chksum += data[i];
		len += sprintf((char*)data+8+size, "%02X", chksum);
		printf("\tWrote %i bytes :\n", len);

		/* SLIP */
		printf("Slipping...\n");
		fmMsgs::serial serout;
		slip_pkg(data, &(serout.data), size+10);

		for (uint32_t i = 0 ; i < serout.data.length() ; i++)
			printf("%02X ", (uint8_t)serout.data.at(i));
		printf("\n");

		//
		printf("Publishing...\n");

		serout.header.seq = seq++;
		serout.header.stamp = ros::Time::now();
		serout.header.frame_id = "AirFrame";
		serout.data.append("\n");
		ser_pub.publish(serout);

		free(data);
		rate.sleep();
	}
}

void serialCallback(const fmMsgs::serial& msg) {
	printf("Serial received...\n");
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

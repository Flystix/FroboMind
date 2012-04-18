/*
 * fmTeleGround.cpp
 *
 *  Created on: Feb 20, 2012
 *      Author: peter
 */

#include <stdint.h>
#include <stdlib.h>
#include <termios.h>

#include <ros/ros.h>
#include <fmMsgs/teleAir2Ground.h>
#include "slip.hpp"
#include "ttySetup.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "fmTeleNode");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	int baudRate;
	std::string device;

	ros::Publisher tele_pub = nh.advertise<fmMsgs::teleAir2Ground> ("/teleData", 1);

	n.param<int> ("baudrate", baudRate, 115200);
	n.param<std::string> ("xBeeDevice", device, "/dev/ttyUSB0");

	int fd = ttySetup(baudRate, device.c_str());

	uint8_t buf[4096];
	char num[9];
	uint32_t msg_type;
	uint32_t msg_size;
	uint8_t msg_chksum;
	while (ros::ok()) {
		uint32_t len = unslip_pkg(fd, buf, 4096); // Only returns when package is received...

		if (len < 8) { // Message is to small - cannot contain header
			tcflush(fd, TCIFLUSH); /* Clean the tty line*/
			continue;
		}

		memcpy(num, buf, 8); // First 8 bytes contain ASCII hex descriptions of msg_type and msg_size
		num[8] = 0x0A; 		 // Insert \n
		sscanf(num, "%04X%04X", &msg_type, &msg_size); // Extract msg_type and msg_size

		if (msg_size != len - 10) {
			tcflush(fd, TCIFLUSH); /* Clean the tty line*/
			continue;
		}

		memcpy(num, &(buf[8 + msg_size]), 2); // Last two bytes contain ASCII hex checksum
		num[2] = 0x0A;						  // Insert \n
		sscanf(num, "%02X", (uint32_t*) &msg_chksum); 	// Extract msg_chksum

		// Calculate check sum
		uint8_t calc_chksum = 0;
		for (uint32_t i = 0; i < len - 2; i++)
			calc_chksum += buf[i];

		if (msg_chksum != calc_chksum) { 	// Check sum mismatch
			tcflush(fd, TCIFLUSH); /* Clean the tty line*/
			continue;
		}

		fmMsgs::teleAir2Ground teleAir2GroundMsg;

		// Do boost / ROS magic to de-serialise message:
		boost::shared_array<uint8_t> bufIn(new uint8_t[msg_size]); // Create boost array of uint8_t's
		ros::serialization::IStream streamIn(bufIn.get(), msg_size); // Create serialisation input stream from buffer
		memcpy(bufIn.get(), buf + 8, msg_size); // Copy content of message into stream

		switch (msg_type) {
			case 0x1000:
				ros::serialization::deserialize(streamIn, teleAir2GroundMsg);
				try{
					tele_pub.publish(teleAir2GroundMsg);
				}catch(ros::serialization::StreamOverrunException e){
					ROS_WARN("StreamOverrunExcetion....");
				}
				break;
			default:
				ROS_WARN("Unknown message type received : %04X", msg_type);
		}
	}
}

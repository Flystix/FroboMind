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

fmMsgs::teleAir2Ground air2gnd;

int main(int argc, char** argv) {
	ros::init(argc, argv, "fmTeleNode");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	int baudRate;
	std::string device;

	ros::Publisher tele_pub = nh.advertise<fmMsgs::teleAir2Ground>("/teleData", 1);

	n.param<int> ("baudrate", baudRate, 115200);
	n.param<std::string> ("device", device, "/dev/ttyUSB0");

	int fd = ttySetup(baudRate, device.c_str());

	uint8_t buf[512];
	char num[9];
	uint32_t msg_type;
	uint32_t msg_size;
	uint8_t msg_chksum;
	while (ros::ok()) {
		while (ros::ok()) {
			uint32_t len = unslip_pkg(fd, buf, 512); // Only returns when package is received...

			if (len < 8) {
				printf("Package to small (len = %i) - skipping\n", len);
				break;
			}

			memcpy(num, buf, 8);							// First 8 bytes contain ascii hex descritions of msg_type and msg_size
			num[8] = 0x0A;									// Insert \n
			sscanf(num, "%04X%04X", &msg_type, &msg_size); 	// Extract msg_type and msg_size

			if (msg_size != len - 10) {
				printf("Package size does not match description: %i (expected %i)\n", len, msg_size + 10);
				break;
			}

			memcpy(num, &(buf[8 + msg_size]), 2);
			num[2] = 0x0A;
			sscanf(num, "%02X", (uint32_t*) &msg_chksum);

			uint8_t calc_chksum = 0;
			for (uint32_t i = 0; i < len - 2; i++)
				calc_chksum += buf[i];

			if (msg_chksum != calc_chksum) {
				printf("Wrong checksum : %i != %i - 10\n", msg_chksum, calc_chksum);
				break;
			}

			fmMsgs::teleAir2Ground msg_in;

			boost::shared_array<uint8_t> bufIn(new uint8_t[msg_size]);
			ros::serialization::IStream streamIn(bufIn.get(), msg_size);
			memcpy(bufIn.get(), buf + 8, msg_size);

			switch(msg_type) {
			case 0x1000:
				ros::serialization::deserialize(streamIn, msg_in);
				tele_pub.publish(msg_in);
				std::cout << msg_in << std::endl;
				break;
			default:
				printf("Unknown message type...\n");
			}
		}
		printf("Returning from error...\n");
		tcflush(fd, TCIFLUSH); /* Clean the tty line*/
	}
}

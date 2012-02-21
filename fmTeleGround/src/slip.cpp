#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <ros/ros.h>

#include "slip.hpp"

void slip_pkg(void* data_in, int fd_out, uint32_t len) {
	uint8_t* data = (uint8_t*)data_in;
	uint8_t out[512];
	uint32_t i;
	out[i++] = STA;
	while (len--) {
		switch(*data) {
		case STA:
			out[i++] = ESC;
			out[i++] = ESC_STA;
			break;
		case END:
			out[i++] = ESC;
			out[i++] = ESC_END;
			break;
		case ESC:
			out[i++] = ESC;
			out[i++] = ESC_ESC;
			break;
		default:
			out[i++] = *data;
		}
		data++;
	}
	out[i++] = END;
	if (write(fd_out, out, i) != i)
		printf("Wrote wrong number of bytes to serial port!");
}

uint32_t unslip_pkg(int fd_in, void* data_out, uint32_t len) {
	uint8_t c = 0;
	uint32_t received = 0;
	uint8_t* data = (uint8_t*)data_out;
	ros::Rate wait(50);
	// Wait for STA
	do{
		while(read(fd_in, &c, 1) <= 0)
			wait.sleep();
	}while(c != STA);

	while(received <= len) {
		while(read(fd_in, &c, 1) <= 0)
			wait.sleep();

		switch(c) {
		case STA:	// Error: Skip invalid package, restart
			received = 0;
			printf("Skipped invalid package : Unexpected STA\n");
			break;
		case END:	// Return package if any
			if (received)
				return received;
			else
				break;
		case ESC:	// Reconstruct escaped character
			while(read(fd_in, &c, 1) <= 0)
				wait.sleep();
			switch(c) {
			case ESC_STA:
				c = STA;
				break;
			case ESC_END:
				c = END;
				break;
			case ESC_ESC:
				c = ESC;
				break;
			default:	// Error: ESC should never occur alone
				printf("Skipped invalid package : ESC %02X\n", c);
				received = 0;
			}
			/* No break */
		default:
			if (received < len)
				data[received++] = c;
		}
	}
	return -1;
}

void slip_pkg(void* data_in, std::string* str_out, uint32_t len) {
//	send_char(END);
	uint8_t* data = (uint8_t*)data_in;
	str_out->clear();
	str_out->append(1, STA);
	while (len--) {
		switch(*data) {
		case STA:
			str_out->append(1, ESC);
			str_out->append(1, ESC_STA);
			break;
		case END:
			str_out->append(1, ESC);
			str_out->append(1, ESC_END);
			break;
		case ESC:
			str_out->append(1, ESC);
			str_out->append(1, ESC_ESC);
			break;
		default:
			str_out->append(1, *data);
		}
		data++;
	}
	str_out->append(1, END);
}

uint32_t unslip_pkg(std::string* str_in, void* data_out, uint32_t len) {
	uint8_t c;
	uint32_t received = 0;
	uint32_t i = 0;
	uint8_t* data = (uint8_t*)data_out;
	// Wait for STA
	do{
		c = str_in->at(i++);
	}while(c != STA);

	while(received <= len) {
		c = str_in->at(i++);
		printf("%i=%02x \n", i, c);
		switch(c) {
		case STA:	// Error: Skip invalid package, restart
			received = 0;
			printf("Skipped invalid package : Unexpected STA\n");
			break;
		case END:	// Return package if any
			if (received)
				return received;
			else
				break;
		case ESC:	// Reconstruct escaped character
			c = str_in->at(i++);
			printf("%i=%02x \n", i, c);
			switch(c) {
			case ESC_STA:
				c = STA;
				break;
			case ESC_END:
				c = END;
				break;
			case ESC_ESC:
				c = ESC;
				break;
			default:	// Error: ESC should never occur alone
				printf("Skipped invalid package : ESC %02X\n", c);
				received = 0;
			}
			/* No break */
		default:
			if (received < len)
				data[received++] = c;
		}
	}
	return -1;
}

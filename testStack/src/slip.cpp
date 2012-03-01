#include <stdint.h>
#include <stdio.h>
#include <iostream>

#include "slip.hpp"

/*
 * Fifo / TTY emulator thing...
 */

uint8_t buf[BUFFER_SIZE];
uint32_t ii  = 0;
uint32_t oi = 0;

void init_fifo(void) {
	uint32_t i = 0;
	while (i < BUFFER_SIZE)
		buf[i++] = 0;
}

void print_fifo(void) {
	uint32_t i = 0;
	while (i < BUFFER_SIZE)
		printf("%02X ", buf[i++]);
	printf("\n");
}

void send_char(uint8_t c) {
	buf[ii] = c;
//	printf("S : %02X \n", c);
	ii = ii >= BUFFER_SIZE ? 0 : ii + 1;
}

uint8_t receive_char(void) {
	uint8_t ret = 0;
	if (1) {
		ret = buf[oi];
		buf[oi] = 0; // For demo purpose
	} else
		printf("No more data!\n");
	oi = oi >= BUFFER_SIZE ? 0 : oi + 1;
//	printf("R : %02X \n", ret);
	return ret;
}

/*
 * SLIP Stuff
 */

void send_pkg(void* p, uint32_t len) {
//	send_char(END);
	uint8_t* data = (uint8_t*)p;
	send_char(STA);
	while (len--) {
		switch(*data) {
		case STA:
			send_char(ESC);
			send_char(ESC_STA);
			break;
		case END:
			send_char(ESC);
			send_char(ESC_END);
			break;
		case ESC:
			send_char(ESC);
			send_char(ESC_ESC);
			break;
		default:
			send_char(*data);
		}
		data++;
	}
	send_char(END);
}

uint32_t receive_pkg(void* p, uint32_t len) {
	uint8_t c;
	uint32_t received = 0;
	uint8_t* data = (uint8_t*)p;
	// Wait for STA
	while(c != STA)
		c = receive_char();

	while(received < len) {
		c = receive_char();
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
			c = receive_char();
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
	if (received != len)
		return -1;
	return received;
}


int slip(const void* data_pkg, int len, uint8_t* slip_pkg) {
	uint8_t* in = (uint8_t*)data_pkg;
	uint8_t* out = (uint8_t*)slip_pkg;

	uint8_t out_i = 0;
	uint8_t in_i = 0;

	while(in_i < len)
		out[out_i++] = in[in_i++];
	printf("\nLen : %i\nin_i : %i\nout_i : %i \n", len, in_i, out_i);
	return out_i;
}

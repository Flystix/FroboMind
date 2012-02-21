#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "ttySetup.hpp"

speed_t get_baud(int baudrate);

int ttySetup(int baudRate, const char* path) {
	struct termios tio;
//	struct termios stdio;
	int tty_fd;
	fd_set rdset;

//	memset(&stdio, 0, sizeof(stdio));
//	stdio.c_iflag = 0;
//	stdio.c_oflag = 0;
//	stdio.c_cflag = 0;
//	stdio.c_lflag = 0;
//	stdio.c_cc[VMIN] = 1;
//	stdio.c_cc[VTIME] = 0;
//	tcsetattr(STDOUT_FILENO, TCSANOW, &stdio);
//	tcsetattr(STDOUT_FILENO, TCSAFLUSH, &stdio);
//	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // make the reads non-blocking

	memset(&tio, 0, sizeof(tio));
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 5;

	tty_fd = open(path, O_RDWR | O_NONBLOCK);
	cfsetospeed(&tio, get_baud(baudRate));
	cfsetispeed(&tio, get_baud(baudRate));

	tcsetattr(tty_fd, TCSANOW, &tio);

	return tty_fd;
}

speed_t get_baud(int baudrate) {
	switch (baudrate) {
	case 0:
		return B0;
	case 50:
		return B50;
	case 75:
		return B75;
	case 110:
		return B110;
	case 134:
		return B134;
	case 150:
		return B150;
	case 200:
		return B200;
	case 300:
		return B300;
	case 600:
		return B600;
	case 1200:
		return B1200;
	case 1800:
		return B1800;
	case 2400:
		return B2400;
	case 4800:
		return B4800;
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	default:
		return B9600;
	}
}

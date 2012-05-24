#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "ttySetup.hpp"

speed_t get_baud(int baudrate);

/**@brief Sets up serial port using termios.
 *
 * The serial port is setup to non blocking read.
 * @param baudRate Desired baudrate (as integer).
 * @param path Path to serial port (/dev/ttyX).
 * @return
 */
int ttySetup(int baudRate, const char* path) {
	struct termios tio;
//	struct termios stdio;
	int tty_fd;
	fd_set rdset;

	memset(&tio, 0, sizeof(tio));
	tio.c_iflag = IGNPAR; /* Ignore bytes with parity errors*/
	tio.c_oflag = 0; /* Raw output */
	tio.c_cflag = CS8 | CREAD | CLOCAL | CRTSCTS;
	tio.c_lflag = 0; /* non- canonical */
	tio.c_cc[VTIME] = 1; /* inter-character timer unused (5) */
	tio.c_cc[VMIN] = 0; /* Dont block */

	tty_fd = open(path, O_RDWR | O_NONBLOCK);
	cfsetospeed(&tio, get_baud(baudRate));
	cfsetispeed(&tio, get_baud(baudRate));

	tcsetattr(tty_fd, TCSANOW, &tio);

	return tty_fd;
}

/**@brief Integer to speed_t conversion.
 * @param baudrate Integer baud rate
 * @return speed_t baudrate.
 */
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

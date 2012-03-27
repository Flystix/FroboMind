 /*
  *
  */

#include "i2cfile.hpp"
#include "i2c-dev.h"
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>

#include <linux/types.h>
#include <sys/ioctl.h>

#include <semaphore.h>

i2cfile::i2cfile(int adapter) {
	char filename[32];
	sprintf(filename, "/dev/i2c-%d", adapter);
	file = open(filename, O_RDWR);
	if (file < 0) {
		ROS_ERROR("I2CFILE : Error opening %s : %s", filename, strerror(errno));
		exit(1);
	} else {
		ROS_INFO("I2CFILE : Opened %s", filename);
	}
	sem_init(&lock, 0, 1);
}

i2cfile::~i2cfile(void) {
	sem_destroy(&lock);
}

int i2cfile::write_byte(int slave, __u8 reg, __u8 data) {
	__u8 buf[2];
	int ret;

	buf[0] = reg;
	buf[1] = data;

	sem_wait(&lock);

	ret = ioctl(file, I2C_SLAVE, slave);
	if (ret < 0) {
		sem_post(&lock);
		ROS_WARN("i2cfile::write_byte : Failed to set %d as slave!\n", slave);
		return ret;
	}

	ret = write(file, buf, 2);
	sem_post(&lock);
	if (ret != 2)
		ROS_WARN("Error writing 0x%02X to reg 0x%02X @ 0x%02X!\n", data, reg, slave);

	return ret;
}
int i2cfile::write_word(int slave, __u8 reg, __u16 data) {
	__u8 buf[3];
	__u8 *data_ptr = (__u8*)&data;
	int ret;

	buf[0] = reg;
	buf[1] = data_ptr[0];
	buf[2] = data_ptr[1];

	sem_wait(&lock);

	ret = ioctl(file, I2C_SLAVE, slave);
	if (ret < 0) {
		sem_post(&lock);
		ROS_WARN("i2cfile::write_word: Failed to slaveess %d!\n", slave);
		return ret;
	}
	ret = write(file, buf, 3);
	sem_post(&lock);
	if (ret != 3)
		ROS_WARN("Error writing 0x%04X to reg 0x%02X @ 0x%02X!\n", data, reg, slave);

	return ret;
}
int i2cfile::write_block(int slave, __u8 reg, void* data, int nBytes) {
	__u8 *buf = (__u8*)malloc(nBytes + 1);
	int ret;

	if (!buf) {
		ROS_WARN("Error allocating %i bytes of memory!\n", nBytes + 1);
		return -1;
	}

	buf[0] = reg;
	for (int i = 0 ; i <= nBytes ; i++)
		buf[i + 1] = ((__u8*)(data))[i];

	sem_wait(&lock);

	ret = ioctl(file, I2C_SLAVE, slave);
	if (ret < 0) {
		sem_post(&lock);
		ROS_WARN("i2cfile::write_byte : Failed to set %d as slave!\n", slave);
		return ret;
	}
	ret = write(file, buf, nBytes + 1);

	sem_post(&lock);
	if (ret != nBytes + 1)
		ROS_WARN("Error writing %i bytes to reg 0x%02X @ 0x%02X!\n", nBytes, reg, slave);

	free(buf);
	return ret;
}
__u8 i2cfile::read_byte(int slave, __u8 reg) {
	__u8 buf;
	int ret;
	sem_wait(&lock);
	ret = ioctl(file, I2C_SLAVE, slave);
	if (ret < 0) {
		sem_post(&lock);
		ROS_WARN("i2cfile::read_byte : Failed to slaveess slave %d!\n", slave);
		return ret;
	}
	ret = write(file, &reg, 1);
	if(ret != 1) {
		sem_post(&lock);
		ROS_WARN("i2cfile::read_byte : Failed to set register %d @ %d!\n", reg, slave);
		return ret;
	}
	ret = read(file, &buf, 1);
	sem_post(&lock);
	if (ret != 1) {
		ROS_WARN("i2cfile::read_byte : Failed to read register %d @ %d!\n", reg, slave);
		return ret;
	}
	return buf;
}
__u16 i2cfile::read_word(int slave, __u8 reg) {
	__u16 buf;
	int ret;
	sem_wait(&lock);
	ret = ioctl(file, I2C_SLAVE, slave);
	if (ret < 0) {
		sem_post(&lock);
		ROS_WARN("i2cfile::read_word : Failed to slaveess slave %d!\n", slave);
		return ret;
	}
	ret = write(file, &reg, 1);
	if(ret != 1) {
		sem_post(&lock);
		ROS_WARN("i2cfile::read_word : Failed to set register %d @ %d!\n", reg, slave);
		return ret;
	}
	ret = read(file, &buf, 2);
	sem_post(&lock);
	if (ret != 2) {
		ROS_WARN("i2cfile::read_word : Failed to read register %d @ %d!\n", reg, slave);
		return ret;
	}
	return buf;
}
int i2cfile::read_block(int slave, __u8 reg, void* data, int nBytes) {
	int ret;

	sem_wait(&lock);
	ret = ioctl(file, I2C_SLAVE, slave);
	if (ret < 0) {
		sem_post(&lock);
		ROS_WARN("i2cfile::read_block : Failed to slaveess slave %d!\n", slave);
		return ret;
	}
	ret = write(file, &reg, 1);
	if(ret != 1) {
		sem_post(&lock);
		ROS_WARN("i2cfile::read_block : Failed to set register %d @ %d!\n", reg, slave);
		return ret;
	}
	ret = read(file, data, nBytes);
	sem_post(&lock);
	if (ret != nBytes)
		ROS_WARN("i2cfile::read_block : Failed to read registers [%d-%d] @ %d!\n", reg, reg+nBytes, slave);

	return ret;
}

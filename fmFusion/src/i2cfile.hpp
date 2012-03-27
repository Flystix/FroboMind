/*
 * i2cfile.hpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */

#ifndef I2CFILE_HPP_
#define I2CFILE_HPP_

#include <semaphore.h>
#include <linux/types.h>

class i2cfile {
public:
	i2cfile(int adapter);
	~i2cfile(void);
	int write_byte(int slave, __u8 reg, __u8 data);
	int write_word(int slave, __u8 reg, __u16 data);
	int write_block(int slave, __u8 reg, void* data, int nBytes);
	__u8 read_byte(int slave, __u8 reg);
	__u16 read_word(int slave, __u8 reg);
	int read_block(int slave, __u8 reg, void* data, int nBytes);
private:
	int file;
	sem_t lock;
};


#endif /* I2CFILE_HPP_ */

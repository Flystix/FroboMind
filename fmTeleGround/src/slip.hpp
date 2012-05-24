#ifndef SLIP_HPP_
#define SLIP_HPP_


#include <stdio.h>
#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <stdint.h>

#define BUFFER_SIZE			64
#define STA					0xC1
#define END					0xCF
#define ESC					0xDB
#define ESC_STA				0xDD
#define ESC_END				0xDC
#define ESC_ESC				0xDE

void slip_pkg(void* data_in, int fd, uint32_t len);
uint32_t unslip_pkg(int fd_in, void* data_out, uint32_t len);

#endif /* SLIP_HPP_ */


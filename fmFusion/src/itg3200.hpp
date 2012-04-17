/*
 * itg3200.hpp
 *
 *  Created on: Feb 7, 2012
 *      Author: peter
 */

#ifndef ITG3200_HPP_
#define ITG3200_HPP_

#include "i2cfile.hpp"
#include <fmMsgs/gyroscope.h>

#define ITG3200_ADDRESS					0x68

#define ITG3200_REG_ID					0x00 /* 110100 */
#define ITG3200_REG_SAMPLE_RATE_DIV		0x15 /* 0x00 */
#define ITG3200_REG_LP_FULL_SCALE		0x16 /* 0x00 */
#define ITG3200_REG_IRQ					0x17 /* 0x00 */
#define ITG3200_REG_IRQ_STATUS			0x1A /* 0x00 */
#define ITG3200_REG_TEMP_OUT_H			0x1B
#define ITG3200_REG_TEMP_OUT_L			0x1C
#define ITG3200_REG_GYRO_XOUT_H			0x1D
#define ITG3200_REG_GYRO_XOUT_L			0x1E
#define ITG3200_REG_GYRO_YOUT_H			0x1F
#define ITG3200_REG_GYRO_YOUT_L			0x20
#define ITG3200_REG_GYRO_ZOUT_H			0x21
#define ITG3200_REG_GYRO_ZOUT_L			0x22
#define ITG3200_REG_POWER_MGMT			0x3E	/* 0x00 */

#define ITG3200_ID_MAGIC				0x69	/* after poweron; can be changed at runtime */

#define ITG3200_FULL_SCALE_2000			(0x03 << 3)

#define ITG3200_LP_256					0x00
#define ITG3200_LP_188					0x01
#define ITG3200_LP_98					0x02
#define ITG3200_LP_42					0x03
#define ITG3200_LP_20					0x04
#define ITG3200_LP_10					0x05
#define ITG3200_LP_5					0x06

#define ITG3200_IRQ_LOGIC_LEVEL				7
#define ITG3200_IRQ_DRIVE_TYPE				6
#define ITG3200_IRQ_LATCH_MODE				5
#define ITG3200_IRQ_LATCH_CLEAR_MODE		4
#define ITG3200_IRQ_DEVICE_READY			2
#define ITG3200_IRC_DATA_AVAILABLE			0

#define ITG3200_IRQ_ACTIVE_LOW				0x01
#define ITG3200_IRQ_ACTIVE_HIGH				0x00
#define ITG3200_IRQ_OPEN_DRAIN				0x01
#define ITG3200_IRQ_PUSH_PULL				0x00
#define ITG3200_IRQ_LATCH_UNTIL_CLEARED		0x01
#define ITG3200_IRQ_LATCH_PULSE				0x00
#define ITG3200_IRQ_ENABLE_DEVICE_READY		0x01
#define ITG3200_IRQ_ENABLE_DATA_AVAILABLE	0x01

#define ITG3200_OSC_INTERNAL				0x00
#define ITG3200_OSC_GYRO_X					0x01
#define ITG3200_OSC_GYRO_Y					0x02
#define ITG3200_OSC_GYRO_Z					0x03
#define ITG3200_OSC_32K						0x04
#define ITG3200_OSC_19M						0x05

#define ITG3200_GYRO_X						(0x01 << 3)
#define ITG3200_GYRO_Y						(0x01 << 4)
#define ITG3200_GYRO_Z						(0x01 << 5)

#define ITG3200_STANDBY_Z					(0x01 << 3)
#define ITG3200_STANDBY_Y					(0x01 << 4)
#define ITG3200_STANDBY_X					(0x01 << 5)
#define ITG3200_SLEEP						(0x01 << 6)
#define ITG3200_RESET						(0x01 << 7)

typedef void (*itg3200_callBackFunc)(const fmMsgs::gyroscope&);

class itg3200 {
public:
	itg3200(i2cfile *i2c_ptr);
	itg3200(i2cfile*, ros::NodeHandle*, ros::Rate, itg3200_callBackFunc);
	~itg3200(void);
//	void get_data(float (*data)[3]);
	void pull();
	void getData(float (*)[3], ros::Time);
	void timerCallback(const ros::TimerEvent&);
	const static __u8 addr = ITG3200_ADDRESS;
private:
	__u8 fullscale;
	__u8 low_pass;
	__u8 powermanagement;
	__s16 bias[3];
	double freq;
	i2cfile *i2c;
	sem_t lock;
	float data[3];
	ros::Time timeStamp;
	itg3200_callBackFunc dataCallback;
	ros::Timer timer;
	ros::NodeHandle* nh;
};

#endif /* ITG3200_HPP_ */

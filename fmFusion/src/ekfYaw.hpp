#ifndef EKF_YAW_H
#define EKF_YAW_H

#include "kalman/ekfilter.hpp"
#include <ros/ros.h>

class ekfYaw : public Kalman::EKFilter<double,1,false,false,true> {
public:
	ekfYaw(double, double, ros::NodeHandle&);
	void updateAttitude(double newPitch, double newRoll);
	void reset(double);
protected:
	void makeBaseA();
	void makeBaseV();
	void makeBaseR();
	void makeBaseQ();

	void makeH();
	void makeW();
	void makeProcess();
	void makeCommonProcess();
	void makeCommonMeasure();
	void makeCommon();
	void makeMeasure();
	void makeDZ();
	double measVar;
	double processVar;
	double theta, phi;
	int newAtt;
	double st, ct, tt;
	double sp, cp, tp;
	double sy, cy, ty;
	/* North compoment : 	17,270.1 nT (+North, -South)
	 * East component : 	   598.0 nT	(+East, -West)
	 * Vertical component : 46,914.0 nT	(+Down, -Up) */
	double B0[3];
};

//typedef ekfYaw::Vector Vector;
//typedef ekfYaw::Matrix Matrix;

#endif

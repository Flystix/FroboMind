#ifndef EKF_YAW_H
#define EKF_YAW_H

#include "kalman/ekfilter.hpp"

class ekfYaw : public Kalman::EKFilter<double,1,false,false,true> {
public:
	ekfYaw(double, double);
	void updateAttitude(double newPitch, double newRoll);
protected:
	void makeBaseA();
	void makeBaseH();
	void makeBaseV();
	void makeBaseR();
	void makeBaseW();
	void makeBaseQ();

	void makeA();
	void makeH();
	void makeW();
	void makeQ();
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

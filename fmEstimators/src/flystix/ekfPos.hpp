#ifndef EKF_POS_H
#define EKF_POS_H

#include "kalman/ekfilter.hpp"

class ekfPos : public Kalman::EKFilter<double,1,false,false,true> {
public:
	ekfPos(double, double);
	void updatePose(double newPitch, double newRoll, double newPos);
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

	double measVar;
	double processVar;
	double theta, phi, psi;
	int newAtt;
	double st, ct, tt;
	double sp, cp, tp;
	double sy, cy, ty;
};

//typedef ekfPos::Vector Vector;
//typedef ekfPos::Matrix Matrix;

#endif

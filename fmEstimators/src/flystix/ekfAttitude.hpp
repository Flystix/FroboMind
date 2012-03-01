#ifndef EKF_PITCH_ROLL_H
#define EKF_PITCH_ROLL_H

#include "kalman/ekfilter.hpp"

class ekfAttitude : public Kalman::EKFilter<double,1,false,false,false> {
public:
	ekfAttitude(double, double);
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
	void makeProcess();
	void makeMeasure();
	void makeDZ();
	double measVar;
	double processVar;
};

//typedef ekfAttitude::Vector Vector;
//typedef ekfAttitude::Matrix Matrix;

#endif

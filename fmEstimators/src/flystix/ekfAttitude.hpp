#ifndef EKF_PITCH_ROLL_H
#define EKF_PITCH_ROLL_H

#include "kalman/ekfilter.hpp"

class ekfAttitude : public Kalman::EKFilter<double,1,false,false,false> {
public:
	ekfAttitude(double, double);
	void updateAngVel(double, double, double);
	void updateAirspeed(double);
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
private:
	double wx, wy, wz;
	double Va;
	static const double g = 9.82;
};

//typedef ekfAttitude::Vector Vector;
//typedef ekfAttitude::Matrix Matrix;

#endif

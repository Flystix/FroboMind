#ifndef EKF_PITCH_ROLL_H
#define EKF_PITCH_ROLL_H

#include "kalman/ekfilter.hpp"

class ekfAtt : public Kalman::EKFilter<double,1,false,false,false> {
public:
	ekfAtt(double, double);
	void updateAirspeed(double newAirspeed);
	void updateAngVel(double newWx, double newWy, double newWz);
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
	double Va, wx, wy, wz;
	static const double g = 9.82;
};

//typedef ekfAttitude::Vector Vector;
//typedef ekfAttitude::Matrix Matrix;

#endif

#ifndef EKF_PITCH_ROLL_H
#define EKF_PITCH_ROLL_H

#define PRECISION 0.0001

#include "kalman/ekfilter.hpp"
#include <fmMsgs/fmVector3.h>

class ekfAttQuat : public Kalman::EKFilter<double,1,false,false,false> {
public:
	ekfAttQuat(double, double);
	void updateAngVel(double, double, double);
	void updateAirspeed(double);
	void updatedt(double);
	Vector getEulerAngles();
	const Vector& getXEuler();
	void reset(double,double);
	fmMsgs::fmVector3 getSim(void);
protected:
	void makeBaseV(); // Measure
	void makeBaseR(); // Measure
	void makeBaseQ(); // Time
//	void makeQ(); // Time

	void makeA(); 		// Time
	void makeH(); 		// Measure
	void makeW(); 		// Time
	void makeProcess(); // Time
	void makeMeasure(); // Measure
	void makeDZ(); 		// ?
	double measVar;
	double processVar;
	Vector xEuler;
	double dt;
private:
	double wx, wy, wz;
	double Va;
	static const double g = 9.82;
};

//typedef ekfAtt::Vector Vector;
//typedef ekfAtt::Matrix Matrix;

#endif

#ifndef EKF_PITCH_ROLL_H
#define EKF_PITCH_ROLL_H

#include "kalman/ekfilter.hpp"

class ekfAtt : public Kalman::EKFilter<double,1,false,false,false> {
public:
	ekfAtt(double, double);
	void updateAngVel(double, double, double);
	void updateAirspeed(double);
	const Vector& getXEuler();
protected:
	void makeBaseA(); // Time
	void makeBaseH(); // Measure
	void makeBaseV(); // Measure
	void makeBaseR(); // Measure
	void makeBaseW(); // Time
	void makeBaseQ(); // Time

	void makeA(); 		// Time
	void makeH(); 		// Measure
	void makeW(); 		// Time
	void makeProcess(); // Time
	void makeMeasure(); // Measure
	void makeDZ(); 		// ?
	double measVar;
	double processVar;
private:
	double wx, wy, wz;
	double Va;
	static const double g = 9.82;
};

//typedef ekfAtt::Vector Vector;
//typedef ekfAtt::Matrix Matrix;

#endif

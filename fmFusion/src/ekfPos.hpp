#ifndef EKF_POS_WIND_H
#define EKF_POS_WIND_H

#include "kalman/ekfilter.hpp"
#define DIFF_PRESS_GAIN 8.064516129

class ekfPos : public Kalman::EKFilter<double,1,false,false,false> {
public:
	ekfPos(double, double, double);
	void reset(double, double, double, double, double);
	double updateVi(double Pd);
protected:
	void makeProcess(); // Process
	void makeBaseA();
	void makeBaseW();
	void makeBaseQ();
	void makeA();
	void makeW();
	void makeQ();

	void makeMeasure(); // Measure
	void makeBaseH();
	void makeBaseV();
	void makeBaseR();
//	void makeH();
//	void makeV();
//	void makeR();

	void makeDZ();

	double gpsVar, altVar;
	double processVar;
	double Vi, Pd, heading;
private:
};

//typedef ekfPos::Vector Vector;
//typedef ekfPos::Matrix Matrix;

#endif /* EKF_POS_WIND_H */

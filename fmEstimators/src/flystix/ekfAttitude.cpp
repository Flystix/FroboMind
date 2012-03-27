
#include "ekfAttitude.hpp"
#include <stdio.h>
#include <cmath>
#include <math.h>
#include <iostream>

using namespace std;

ekfAttitude::ekfAttitude(double processVariance, double measurementVariance)  {
//	setSizeX(2);	//!< Size of the state vector.
//	setSizeU(3);	//!< Size of the input vector.
//	setSizeW(2);	//!< Size of the process noise vector. Matrix?!
//	setSizeZ(3);	//!< Size of the measurement vector.
//	setSizeV(3); 	//!< Size of the measurement noise vector. Matrix?!
	setDim(2, 3, 3, 3, 3);
	measVar = measurementVariance;
	processVar = processVariance;
}

void ekfAttitude::makeDZ() {
	for (int i = 1 ; i <= 2 ; i++) {
		/* PI-wrap... */
		while (x(i) >  M_PI)
			x(i) -= 2 * M_PI;
		while (x(i) < -M_PI)
			x(i) += 2 * M_PI;
		/* Limit pitch & roll to +/-85 degrees to avoid singularities */
		if (x(i) >  85*M_PI/180)
			x(i) =  85*M_PI/180;
		if (x(i) < -85*M_PI/180)
			x(i) = -85*M_PI/180;
	}
}

void ekfAttitude::makeBaseA() {
//	A(1,1) = tt*(q*cp - r*sp);
//	A(1,2) = (q*sp + r*cp)/(ct*ct);
//	A(2,1) = -q*sp - r*cp;
	A(2,2) = 0;
}

void ekfAttitude::makeA() {
	double cp = cos(x(1));
	double sp = sin(x(1));
	double ct = cos(x(2));
	double tt = tan(x(2));

	double q = u(2), r = u(3);
	A(1,1) = tt*(q*cp - r*sp);
	A(1,2) = (q*sp + r*cp)/(ct*ct);
	A(2,1) = -q*sp - r*cp;
//	A(2,2) = 0;
}

void ekfAttitude::makeBaseW() {
	W(1,1) = 1;
//	W(1,2) = tt * sp;
//	W(1,3) = tt * cp;
	W(2,1) = 0;
//	W(2,2) = cp;
//	W(2,3) = -sp;
}

void ekfAttitude::makeW() {
	double cp = cos(x(1));
	double sp = sin(x(1));
	double tt = tan(x(2));
//	W(1,1) = 1;
	W(1,2) = tt * sp;
	W(1,3) = tt * cp;
//	W(2,1) = 0;
	W(2,2) = cp;
	W(2,3) = -sp;
}

void ekfAttitude::makeBaseQ() {
	Q(1,1) =  1.00000 * processVar;
	Q(1,2) = -0.17227 * processVar;
	Q(1,3) =  0.11208 * processVar;
	Q(2,1) = -0.17227 * processVar;
	Q(2,2) =  1.00000 * processVar;
	Q(2,3) = -0.25561 * processVar;
	Q(3,1) =  0.11208 * processVar;
	Q(3,2) = -0.25561 * processVar;
	Q(3,3) =  1.00000 * processVar;
}

void ekfAttitude::makeBaseH() {
	H(1,1) =  0;
//	H(1,2) =  ct;
//	H(2,1) = -ct * cp;
//	H(2,2) =  sp * st;
//	H(3,1) =  sp * ct;
//	H(3,2) =  cp * st;
}

void ekfAttitude::makeH() {
	double cp = cos(x(1));
	double sp = sin(x(1));
	double ct = cos(x(2));
	double st = sin(x(2));
//	H(1,1) =  0;
	/*
	 * [                  0,                        g*cos(th) + Va*wx*cos(th)]
	 * [ -g*cos(ph)*cos(th), g*sin(ph)*sin(th) - Va*(wx*cos(th) + wz*sin(th))]
	 * [  g*cos(th)*sin(ph),                g*cos(ph)*sin(th) - Va*wy*cos(th)]
	 */
	H(1,2) =  g*ct    + Va*wx*ct;
	H(2,1) = -g*ct*cp;
	H(2,2) =  g*sp*st - Va*(wx*ct + wz*st);
	H(3,1) =  g*sp*ct;
	H(3,2) =  g*cp*st - Va*wx*ct;
}

void ekfAttitude::makeBaseV() {
	V(1,1) = 1.0;
	V(1,2) = 0.0;
	V(1,3) = 0.0;
	V(2,1) = 0.0;
	V(2,2) = 1.0;
	V(2,3) = 0.0;
	V(3,1) = 0.0;
	V(3,2) = 0.0;
	V(3,3) = 1.0;
}

void ekfAttitude::makeBaseR() {
	R(1,1) =  1.000000 * measVar;
	R(2,1) =  0.480250 * measVar;
	R(3,1) = -0.041471 * measVar;
	R(1,2) =  0.480250 * measVar;
	R(2,2) =  1.000000 * measVar;
	R(3,2) = -0.111440 * measVar;
	R(1,3) = -0.041471 * measVar;
	R(2,3) = -0.111440 * measVar;
	R(3,3) =  1.000000 * measVar;
}

void ekfAttitude::makeProcess() { // Implements f(x,u,0)
	Vector x_(x.size());
	double p = u(1), q = u(2), r = u(3);
	double cp = cos(x(1));
	double sp = sin(x(1));
	double tt = tan(x(2));
	x_(1) = x(1) + p + tt*(sp*q + cp*r);
	x_(2) = x(2) + cp*q - sp*r;
	x.swap(x_);
}

void ekfAttitude::makeMeasure() { // Implements h(x,0)
	double cp = cos(x(1));
	double sp = sin(x(1));
	double ct = cos(x(2));
	double st = sin(x(2));
	Vector z_(z.size());

	/* Gravitational vector [m/s²]*/
	z_(1) =     st*g;
	z_(2) = -sp*ct*g;
	z_(3) = -cp*ct*g;
	/* Centripetal forces [m/s] * [rad/s] = [m/s²]*/
	z_(1) +=  Va*(wy*st);
	z_(2) +=  Va*(wz*ct - wx*st);
	z_(3) += -Va*(wy*ct);
	z.swap(z_);
}

void ekfAttitude::updateAngVel(double newWx, double newWy, double newWz) {
	wx = newWx;
	wy = newWy;
	wz = newWz;
}
void ekfAttitude::updateAirspeed(double newAirspeed) {
	Va = newAirspeed;
}

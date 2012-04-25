
#include "ekfPos.hpp"
#include <stdio.h>
#include <cmath>
#include <math.h>
#include <iostream>
#include <ros/ros.h>

using namespace std;

#define DEBUG	0

#if DEBUG == 1
// #define DEBUG_STR(str)	ROS_INFO(str);
#define DEBUG_STR(str)	ROS_INFO(str);
#else
#define DEBUG_STR(str)	;
#endif

ekfPos::ekfPos(double processVariance, double gpsVariance, double altVariance)  {
//	setSizeX(2);	//!< Size of the state vector (x).
//	setSizeU(3);	//!< Size of the input vector (u).
//	setSizeW(2);	//!< Size of the process noise vector (w -> Q). Matrix?!
//	setSizeZ(3);	//!< Size of the measurement vector (z).
//	setSizeV(3); 	//!< Size of the measurement noise vector (v -> R). Matrix?!
	setDim(8, 4, 9, 3, 3);
	altVar = altVariance;
	gpsVar = gpsVariance;
	processVar = processVariance;
	heading = 0;
	DEBUG_STR("ekfPos : ekfPos initialized...");
}

void ekfPos::makeProcess() { // Implements f(x,u,0)
	DEBUG_STR("ekfPos : makeProcess...");
	/*
	 * f =
	 *  Pn - dt*(Wn - cos(ps)*cos(th)*(aPd*(Pd - bPd))^(1/2))
	 *  Pe - dt*(We - cos(th)*sin(ps)*(aPd*(Pd - bPd))^(1/2))
	 *                    Wn
	 *                    We
	 *                    aPd
	 *                    bPd
	 *  Alt + dt*sin(th - alfa)*(aPd*(Pd - bPd))^(1/2)
	 *                    alfa
	 */
	Vector x_(x.size());
	double th = u(1);
	double ps = u(2);
	double Pd = u(3);
	double dt = u(4);
	heading = ps;

	double Wn = x(3);
	double We = x(4);
	double aPd = x(5);
	double bPd = x(6);
	double alfa = x(8);

	if (isnan(Pd))
		Pd = 0.0;
	if (Pd - bPd < 0.1)
		bPd = Pd - 0.1;
	if (aPd < 0.1)
		aPd = 0.1;

	Vi = sqrt(aPd * (Pd - bPd));
	x_(1) = x(1) + dt * (cos(th - alfa) * cos(ps) * Vi - Wn);
	x_(2) = x(2) + dt * (cos(th - alfa) * sin(ps) * Vi - We);
	x_(3) = x(3);
	x_(4) = x(4);
	x_(5) = x(5);
	x_(6) = x(6);
	x_(7) = x(7) + dt * sin(th - alfa) * Vi;
	x_(8) = x(8);
	x.swap(x_);
	DEBUG_STR("ekfPos : done");
}

void ekfPos::makeBaseA() {
	DEBUG_STR("ekfPos : makeBaseA...");
	for (unsigned int i = 1 ; i <= A.nrow() ; i++)
		for(unsigned int j = 1 ; j <= A.ncol() ; j++)
			A(i,j) = (i == j ? 1.0 : 0.0);
	DEBUG_STR("ekfPos : done");
}
void ekfPos::makeA() {
	DEBUG_STR("ekfPos : makeA...");
	double th = u(1);
	double ps = u(2);
	double Pd = u(3);
	double dt = u(4);
	double aPd = x(5);
	double bPd = x(6);
	double alfa = x(8);

	if (isnan(Pd))
		Pd = 0.0;
	if (Pd - bPd < 0.1)
		bPd = Pd - 0.1;
	if (aPd < 0.1)
		aPd = 0.1;

	Vi = sqrt(aPd * (Pd - bPd));

	A(1,3) =  -dt;
	A(2,4) =  -dt;
//	          (dt * cos(th - alfa) * cos(ps) * (Pd - bPd)) / (2 * (aPd*(Pd - bPd))^(1/2))
	A(1,5) =  (dt * cos(th - alfa) * cos(ps) * (Pd - bPd)) / (2 * Vi);
//	          (dt * cos(th - alfa) * sin(ps) * (Pd - bPd)) / (2 * (aPd*(Pd - bPd))^(1/2))
	A(2,5) =  (dt * cos(th - alfa) * sin(ps) * (Pd - bPd)) / (2 * Vi);
//	         -(dt * cos(th - alfa) * cos(ps) * aPd) / (2 * (aPd*(Pd - bPd))^(1/2))
	A(1,6) = -(dt * cos(th - alfa) * cos(ps) * aPd) / (2 * Vi);
//	         -(dt * cos(th - alfa) * sin(ps) * aPd) / (2 * (aPd * (Pd - bPd))^(1/2))
	A(2,6) = -(dt * cos(th - alfa) * sin(ps) * aPd) / (2 * Vi);
//		      (dt*sin(th - alfa + wTh)*(Pd - bPd)) / (2 * (aPd * (Pd - bPd))^(1/2))
	A(7,5) =  (dt * sin(th - alfa) * (Pd - bPd)) / (2*Vi);
//           -(aPd * dt * sin(th - alfa)) / (2 * (aPd * (Pd - bPd))^(1/2))
	A(7,6) = -(aPd * dt * sin(th - alfa)) / (2 * Vi);
//			 -dt * cos(th - alfa) * (aPd * (Pd - bPd))^(1/2)
	A(7,8) = -dt * cos(th - alfa) * Vi;
//	          dt * sin(th - alfa) * cos(ps) * (aPd * (Pd - bPd))^(1/2)
	A(1,8) =  dt * sin(th - alfa) * cos(ps) * Vi;
//	          dt * sin(th - alfa) * sin(ps) * (aPd * (Pd - bPd))^(1/2)
	A(2,8) =  dt * sin(th - alfa) * sin(ps) * Vi;
	for(unsigned int i = 1 ; i <= A.ncol() ; i++)
		A(i,i) = 1.0;
}

void ekfPos::makeBaseW() {
	DEBUG_STR("ekfPos : makeBaseW...");
	for (unsigned int i = 1 ; i <= W.nrow() ; i++)
		for(unsigned int j = 1 ; j <= W.ncol() ; j++)
			W(i,j) = 0.0;
	W(3,4) = 1.0;
	W(4,5) = 1.0;
	W(5,6) = 1.0;
	W(6,7) = 1.0;
	W(7,8) = 1.0;
	W(8,9) = 1.0;
	DEBUG_STR("ekfPos : done");
}
void ekfPos::makeW() {
	double th = u(1);
	double ps = u(2);
	double Pd = u(3);
	double dt = u(4);
	double aPd = x(5);
	double bPd = x(6);
	double alfa = x(8);

	if (isnan(Pd))
		Pd = 0.0;
	if (Pd - bPd < 0.1)
		bPd = Pd - 0.1;
	if (aPd < 0.1)
		aPd = 0.1;

	Vi = sqrt(aPd * (Pd - bPd));
//           (aPd * dt * cos(ps) * cos(th - alfa)) / (2 * (aPd * (Pd - bPd))^(1/2))
	W(1,1) = (aPd * dt * cos(th - alfa) * cos(ps)) / (2 * Vi);

//	         -dt * sin(ps) * cos(th - alfa) * (aPd * (Pd - bPd))^(1/2)
	W(1,2) = -dt * cos(th - alfa) * sin(ps) * Vi;

//	         -dt * sin(th - alfa) * cos(ps) * (aPd*(Pd - bPd))^(1/2)
	W(1,3) = -dt * sin(th - alfa) * cos(ps) * Vi;

//	         (aPd * dt * sin(ps) * cos(th - alfa)) / (2 * (aPd*(Pd - bPd))^(1/2))
	W(2,1) = (aPd * dt * cos(th - alfa) * sin(ps)) / (2 * Vi);

//	          dt * cos(ps) * cos(th - alfa) * (aPd * (Pd - bPd))^(1/2)
	W(2,2) =  dt * cos(th - alfa) * cos(ps) * Vi;

//	         -dt * sin(th - alfa) * sin(ps) * (aPd * (Pd - bPd))^(1/2)
	W(2,3) = -dt * sin(th - alfa) * sin(ps) * Vi;

//	         (aPd * dt * sin(th - alfa)) / (2 * (aPd*(Pd - bPd))^(1/2))
	W(7,1) = (aPd * dt * sin(th - alfa)) / (2 * Vi);
//	         dt * cos(th - alfa) * (aPd * (Pd - bPd))^(1/2)
	W(7,3) = dt * cos(th - alfa) * Vi;
}

void ekfPos::makeBaseQ() {
	for (unsigned int i = 1 ; i <= Q.nrow() ; i++)
		for(unsigned int j = 1 ; j <= Q.ncol() ; j++)
			Q(i,j) = (i == j ? 1 : 0);
}

void ekfPos::makeQ() { // Process noise covariance matrix
	DEBUG_STR("ekfPos : makeQ...");
	Q(1,1) = 0.1; // wPd
	Q(2,2) = 0.01; // wPsi
	Q(3,3) = 0.01; // wTh
	Q(4,4) = 0.0001; // wWn
	Q(5,5) = 0.0001; // wWe
	Q(6,6) = 0.000001; // wAPd
	Q(7,7) = 0.000001; // wBPd
	Q(8,8) = 0.01; // wAlt
	Q(9,9) = 0.0001; // wAlfa
	DEBUG_STR("ekfPos : Done");
}

void ekfPos::makeMeasure() { // Implements h(x,0)
	DEBUG_STR("ekfPos : makeMeasure...");
	/*
	 * h =
	 *  Pn
	 *  Pe
	 */
	Vector z_(z.size());
	z_(1) = x(1);
	z_(2) = x(2);
	z_(3) = x(7);
	z.swap(z_);
	DEBUG_STR("ekfPos : done");
}

void ekfPos::makeBaseH() {
	DEBUG_STR("ekfPos : makeBaseH...");
	/*
	 * H =
	 * [ 1, 0, 0, 0, 0, 0, 0, 0]
	 * [ 0, 1, 0, 0, 0, 0, 0, 0]
	 * [ 0, 0, 0, 0, 0, 0, 1, 0]
	 */
	for (unsigned int i = 1 ; i <= H.nrow() ; i++)
		for(unsigned int j = 1 ; j <= H.ncol() ; j++)
			H(i,j) = (0.0);
	H(1,1) = 1.0;
	H(2,2) = 1.0;
	H(3,7) = 1.0;
	DEBUG_STR("ekfPos : done");
}


void ekfPos::makeBaseV() {
	DEBUG_STR("ekfPos : makeBaseV...");
	/* V =
	 *  [ 1, 0]
	 * 	[ 0, 1]
	 */
	for (unsigned int i = 1 ; i <= V.nrow() ; i++)
		for(unsigned int j = 1 ; j <= V.ncol() ; j++)
			V(i,j) = (i == j ? 1.0 : 0.0);
	DEBUG_STR("ekfPos : done...");
}

void ekfPos::makeBaseR() { // Measurement noise covariance matrix
	for (unsigned int i = 1 ; i <= R.nrow() ; i++)
		for(unsigned int j = 1 ; j <= R.ncol() ; j++)
			R(i,j) = 0.0;
	DEBUG_STR("ekfPos : makeBaseR...");
	R(1,1) = gpsVar;
	R(2,2) = gpsVar;
	R(3,3) = altVar;
	DEBUG_STR("ekfPos : done...");
}

void ekfPos::makeDZ() {
	if (x(5) < 0.1)
		x(5) = 0.1;
	if (x(6) < 0.1)
		x(6) = 0.1;
	x(8) = fmod(x(8), 2*M_PI);
}

void ekfPos::reset(double initPn, double initPe, double initWn, double initWe, double initRho = 1.2250) {
	double P0[] = { 10.0, 0.0, 0.0, 0.0,  0.0,     0.0, 0.0,    0.0,
		            0.0, 10.0, 0.0, 0.0,  0.0,     0.0, 0.0,    0.0,
					0.0, 0.0,  0.01, 0.0, 0.0,     0.0, 0.0,    0.0,
					0.0, 0.0,  0.0, 0.01, 0.0,     0.0, 0.0,    0.0,
					0.0, 0.0,  0.0, 0.0,  0.0001,  0.0, 0.0,    0.0,
					0.0, 0.0,  0.0, 0.0,  0.0,     1.0, 0.0,    0.0,
					0.0, 0.0,  0.0, 0.0,  0.0,     0.0, 1000.0, 0.0,
					0.0, 0.0,  0.0, 0.0,  0.0,     0.0, 0.0,    0.1,};
	double x0[] = { initPn, initPe, initWn, initWe , 8.82, 3.91, 0.0, 0.0}; // 10.60, 1.92
	Vector xInit(8, x0);
	Matrix PInit(8, 8, P0);
	init(xInit, PInit);
}

double ekfPos::updateVi(double rawPd) {
	double aPd = x(5);
	double bPd = x(6);
	Pd = rawPd;
	if (Pd - bPd < 0) {
		Pd = bPd + 0.1;
	}
	Vi = sqrt(aPd * (Pd - bPd));
	if (isnan(Vi) || Vi < 0)
		Vi = 0.0;
	return Vi;
}

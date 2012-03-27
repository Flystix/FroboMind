
#include "ekfPos.hpp"
#include <stdio.h>
#include <cmath>
#include <math.h>
#include <iostream>

using namespace std;

/* CONSTRUCTORS */

ekfPos::ekfPos(double processVariance, double measurementVariance)  {
//	setSizeX(5);	//!< Size of the state vector. (Pn Pe Wn We Va)
//	setSizeU(1);	//!< Size of the input vector. (dt)
//	setSizeW(5);	//!< Size of the process noise vector. (wPn, wPe, wWn, wWe, wVa)
//	setSizeZ(3);	//!< Size of the measurement vector. (GPSn, GPSe, Vg)
//	setSizeV(3); 	//!< Size of the measurement noise vector. (vGPSn, vGPSe, vVg)
	setDim(5, 1, 5, 3, 3);
	measVar = measurementVariance;
	processVar = processVariance;
	theta = 0;
	phi = 0;
	psi = 0;
}

/* JACOBIANS (A, W, H & V) */

void ekfPos::makeBaseA() { 	/* df/dx (x,u,0) = 5x5 */
	/*
	 * +-                                    -+
	 * |  0, 0, -dt,  0,  dt cos(ps) cos(th)  |
	 * |  0, 0,  0,  -dt, dt cos(th) sin(ps)  |
	 * |  0, 0,  0,   0,           0          |
	 * |  0, 0,  0,   0,           0          |
	 * |  0, 0,  0,   0,           0          |
	 * +-                                    -+
	 */
	for (int i = 1 ; i <= 5 ; i++)
		for (int j = 1 ; j <= 5 ; j++)
			A(i,j) = 0;
}

void ekfPos::makeA() {		/* df/dx (x,u,0) = 5x5 */
	double dt = u(1);
	A(1,3) = -dt;
	A(1,5) =  dt * cy * ct;
	A(2,4) = -dt;
	A(2,5) =  dt * ct * sy;
}

void ekfPos::makeBaseW() {	/* df/dw (x,u,0) = 1x2 */
	/*
	 *  +-                   -+
	 *  |  1, 0, -dt,  0,  0  |
	 *  |  0, 1,  0,  -dt, 0  |
	 *  |  0, 0,  1,   0,  0  |
	 *  |  0, 0,  0,   1,  0  |
	 *  |  0, 0,  0,   0,  1  |
	 *  +-                   -+
	 */

	for (int i = 1 ; i <= 5 ; i++)
		for (int j = 1 ; j <= 5 ; j++)
			W(i,j) = (i == j ? 1 : 0);
}

void ekfPos::makeW() {		/* df/dw (x,u,0) = 1x2 */
//	double Wn = x(3);
//	double We = x(4);
//	double Va = x(5);
	double dt = u(1);
//	W(1,1) = Va * ct * cy - Wn;
//	W(2,1) = Va * ct * sy - We;
//	W(3,2) = 1;
//	W(4,3) = 1;
//	W(5,4) = 1;
	W(1,3) = -dt;
	W(2,4) = -dt;
}

void ekfPos::makeBaseH() {	/* dh/dx (x,0) = 3x5 */
	/*  +-                     -+
	 *  |  1, 0, 0, 0,    0     |
	 *  |  0, 1, 0, 0,    0     |
	 *  |  0, 0, 0, 0, cos(th)  |
	 *  +-                     -+
	 */
	for (int i = 1 ; i <= 3 ; i++)
		for (int j = 1 ; j <= 5 ; j++)
			H(i,j) = (i == j ? 1 : 0);
	H(3,3) = 0;
}

void ekfPos::makeH() {		/* dh/dx (x,0) = 3x5 */
	H(3,5) = ct;
}

void ekfPos::makeBaseV() { 	/* dh/dv (x,0) = 3x3 */
	/*
	 *   +-         -+
	 *   |  1, 0, 0  |
	 *   |  0, 1, 0  |
	 *   |  0, 0, 1  |
	 *   +-         -+
	 */
	for (int i = 1 ; i <= 3 ; i++)
		for (int j = 1 ; j <= 3 ; j++)
			V(i,j) = (i == j ? 1 : 0);
}

/* COVARIANCE MATRICES (Q & R)*/

void ekfPos::makeBaseQ() {	/* Process noise covariance = 5x5 */
	// printf("makeBaseQ...\n");
	for (int i = 1 ; i <= 4 ; i++)
		for (int j = 1 ; j <= 4 ; j++) {
			printf("makeBaseQ : [i,j] = [%i,%i]\n", i,j);
			Q(i,j) = (i == j ? processVar : 0);
		}

}

void ekfPos::makeQ() {
	double QOT = 100;
	double QXT = 20;
	Q(1,1) = abs(cy)*QOT + abs(sy)*QXT;
	Q(2,2) = abs(sy)*QOT + abs(cy)*QXT;
}

void ekfPos::makeBaseR() {	/* Measurement noise covariance matrix*/
	// printf("makeBaseR...\n");
	for (int i = 1 ; i <= 3 ; i++)
		for (int j = 1 ; j <= 3 ; j++)
			R(i,j) = (i == j ? measVar : 0);
}

/* UPDATES - f(x,u,w) & h(x,v) */

void ekfPos::makeProcess() { // Implements f(x,u,0)
	/*
	 *   +-                               -+
	 *   |  -dt (Wn - Va cos(ps) cos(th))  |
	 *   |  -dt (We - Va cos(th) sin(ps))  |
	 *   |                0                |
	 *   |                0                |
	 *   |                0                |
	 *   +-                               -+
	 */
	Vector x_(x.size());
	double dt = u(1);
	double Wn = x(3);
	double We = x(4);
	double Va = x(5);
	x_(1) = x(1);// - dt * (Wn - Va * cy * ct);
	x_(2) = x(2);// - dt * (We - Va * sy * ct);
	x_(3) = x(3) + 0;
	x_(4) = x(4) + 0;
	x_(5) = x(5) + 0;
	x.swap(x_);
}

void ekfPos::makeMeasure() { // Implements h(x,u) (Sensor prediction)
	/*
	 *   +-            -+
	 *   |      Pn      |
	 *   |      Pe      |
	 *   |  Va cos(th)  |
	 *   +-            -+
	 */
	Vector z_(z.size());
	double Pn = x(1);
	double Pe = x(2);
	double Va = x(5);
	z_(1) = Pn;
	z_(2) = Pe;
	z_(3) = Va * ct;
	z.swap(z_);
}

/* DATA UPDATES */

void ekfPos::updatePose(double newPhi, double newTheta, double newPsi) {
	// printf("updateAttitude...\n");
	phi = newPhi;
	theta = newTheta;
	psi = newPsi;
	newAtt = 1;
}


void ekfPos::makeCommon() {
	if (!newAtt)
		return;
	cp = cos(phi);
	sp = sin(phi);
	tp = tan(phi);
	ct = cos(theta);
	st = sin(theta);
	tt = tan(theta);
	cy = cos(psi);
	sy = sin(psi);
	ty = tan(psi);
	newAtt = 0;
}

void ekfPos::makeCommonMeasure() {
	// printf("makeCommonMeasure...\n");
	makeCommon();
}
void ekfPos::makeCommonProcess() {
	// printf("makeCommonProcess...\n");
	makeCommon();
}

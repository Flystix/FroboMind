
#include "ekfYaw.hpp"
#include <stdio.h>
#include <cmath>
#include <math.h>
#include <iostream>

using namespace std;

/* CONSTRUCTORS */

ekfYaw::ekfYaw(double processVariance, double measurementVariance)  {
	// printf("ekfYaw(%2.2,%2.2)...\n", processVariance, measurementVariance);
//	setSizeX(2);	//!< Size of the state vector. (Yaw)
//	setSizeU(3);	//!< Size of the input vector. (wy, wz)
//	setSizeW(2);	//!< Size of the process noise vector. (2)
//	setSizeZ(3);	//!< Size of the measurement vector. (Magnetometer)
//	setSizeV(3); 	//!< Size of the measurement noise vector. (Magnetometer noise)
	// printf("Initializing...");
	setDim(1, 2, 2, 3, 3);
	measVar = measurementVariance;
	processVar = processVariance;
//	B0[0] =  17.2701; // [nT]
//	B0[1] =   0.5980; // [nT]
//	B0[2] =  46.9140; // [nT]
	B0[0] =  0.172701; // [G]
	B0[1] =  0.005980; // [G]
	B0[2] =  0.469140; // [G]
	theta = 0;
	phi = 0;
	// printf("Done");
}

/* JACOBIANS (A, W, H & V) */

void ekfYaw::makeBaseA() { 	/* df/dx (x,u,0) = 1x1 */
	// printf("makeBaseA...\n");
	A(1,1) = 0;
}

void ekfYaw::makeA() {		/* df/dx (x,u,0) = 1x1 */
	/* Empty */
	// printf("makeA...\n");
}

void ekfYaw::makeBaseW() {	/* df/dw (x,u,0) = 1x2 */
	/* Empty */
	// printf("makeBaseW...\n");
}

/* TODO: Fix me */
void ekfYaw::makeW() {		/* df/dw (x,u,0) = 1x2 */
	// printf("makeW...\n");

//	double wy = u(1);
//	double wz = u(2);

//	W(1,1) =  wy * cp / ct - wz * sp / ct;
//	W(1,2) = (wy * sp + wz * cp) * st / (ct * ct);
	W(1,1) = sp / ct;
	W(1,2) = cp / ct;
}

void ekfYaw::makeBaseH() {	/* dh/dx (x,0) = 3x1 */
	/* Empty */
	// printf("makeBaseH...\n");
}

void ekfYaw::makeH() {		/* dh/dx (x,0) = 3x1 */
	// printf("makeH...\n");

	H(1,1) = -ct * sy * B0[0] +
			  ct * cy * B0[1];

	H(2,1) = (-sp * st * sy - cp * cy) * B0[0] +
			 ( sp * st * cy - cp * sy) * B0[1];

	H(3,1) = (-cp * st * sy + sp * cy) * B0[0] +
			 ( cp * st * cy + sp * sy) * B0[1];
}

void ekfYaw::makeBaseV() { 	/* dh/dv (x,0) = 3x3 */
	// printf("makeBaseV...\n");
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

/* COVARIANCE MATRICES (Q & R)*/

void ekfYaw::makeBaseQ() {	/* Process noise covariance = 2x2 */
	// printf("makeBaseQ...\n");
	Q(1,1) =  1.00000 * processVar;
	Q(1,2) =  0.0;
	Q(2,1) =  0.0;
	Q(2,2) =  1.00000 * processVar;

}

void ekfYaw::makeQ() {		/* Process noise covariance = 2x2 */
	/* Empty */
	// printf("makeQ...\n");
}

void ekfYaw::makeBaseR() {	/* Measurement noise covariance matrix*/
	// printf("makeBaseR...\n");
	R(1,1) =  1.000000 * measVar;
	R(1,2) =  0.0;
	R(1,3) =  0.0;
	R(2,1) =  0.0;
	R(2,2) =  1.000000 * measVar;
	R(2,3) =  0.0;
	R(3,1) =  0.0;
	R(3,2) =  0.0;
	R(3,3) =  1.000000 * measVar;
}

/* UPDATES - f(x,u,w) & h(x,v) */

void ekfYaw::makeProcess() { // Implements f(x,u,0)
	// printf("makeProcess...\n");
	Vector x_(x.size());
	double wy = u(1);
	double wz = u(2);
	x_(1) = x(1) + wy * (sp / ct) + wz * (cp / ct);
	x.swap(x_);
}

void ekfYaw::makeMeasure() { // Implements h(x,u) (Sensor prediction)
	// printf("makeMeasure...\n");
	Vector z_(z.size());

	z_(1) = ct * cy * B0[0] +
			ct * sy * B0[1] -
			st * B0[2];
	z_(2) = (sp * st * cy - cp * sy) * B0[0] +
			(sp * st * sy + cp * cy) * B0[1] +
			sp * ct * B0[2];
	z_(3) = (cp * st * cy + sp * sy) * B0[0] +
			(cp * st * sy - sp * cy) * B0[1] +
			cp * ct * B0[2];

	z.swap(z_);
}

/* DATA UPDATES */

void ekfYaw::updateAttitude(double newPhi, double newTheta) {
	// printf("updateAttitude...\n");
	phi = newPhi;
	theta = newTheta;
	newAtt = 1;
}


void ekfYaw::makeCommon() {
	if (!newAtt)
		return;
	// printf("makeCommon...\n");
	cp = cos(phi);
	sp = sin(phi);
	tp = tan(phi);
	ct = cos(theta);
	st = sin(theta);
	tt = tan(theta);
	newAtt = 0;
}

void ekfYaw::makeCommonMeasure() {
	// printf("makeCommonMeasure...\n");
	makeCommon();
	cy = cos(x(1));
	sy = sin(x(1));
	ty = tan(x(1));
}
void ekfYaw::makeCommonProcess() {
	// printf("makeCommonProcess...\n");
	makeCommon();
	cy = cos(x(1));
	sy = sin(x(1));
	ty = tan(x(1));
}

void ekfYaw::makeDZ() {
	// printf("makeDZ...\n");
	while (x(1) >  M_PI)
		x(1) -= 2 * M_PI;
	while (x(1) < -M_PI)
		x(1) += 2 * M_PI;
}

#include "ekfAttQuat.hpp"
#include <stdio.h>
#include <cmath>
#include <math.h>
#include <iostream>
#include <ros/ros.h>

#define DEBUG 	1

#if DEBUG == 1
#define DEBUG_STR(str)	ROS_INFO(str);
#else
#define DEBUG_STR(str)	;
#endif

using namespace std;

ekfAttQuat::ekfAttQuat(double processVariance, double measurementVariance) {
	//	setSizeX(2);	//!< Size of the state vector.
	//	setSizeU(3);	//!< Size of the input vector.
	//	setSizeW(2);	//!< Size of the process noise vector. (1x4?)
	//	setSizeZ(3);	//!< Size of the measurement vector.
	//	setSizeV(3); 	//!< Size of the measurement noise vector.
	setDim(4, 3, 3, 3, 3);
	measVar = measurementVariance;
	processVar = processVariance;
}

void ekfAttQuat::makeDZ() {
	DEBUG_STR("ekfAttQuat : make DZ...");
	// Normalize quaternion
	double var = 0;
	for (int i = 1; i <= 4; i++)
		var += x(i) * x(i);

	if (var < (1 + PRECISION) && var > (1 - PRECISION))
		return; // Dont normalize - Quaternion is within limits

	var = sqrt(var);
	for (int i = 1; i <= 4; i++)
		x(i) /= var;
	DEBUG_STR("ekfAttQuat : Done!");
}

void ekfAttQuat::makeBaseA() {
	DEBUG_STR("ekfAttQuat : makeBaseA...");
	//	A(1,1) = tt*(q*cp - r*sp);
	//	A(1,2) = (q*sp + r*cp)/(ct*ct);
	//	A(2,1) = -q*sp - r*cp;
	A(2, 2) = 0;
	DEBUG_STR("ekfAttQuat : done");
}

void ekfAttQuat::makeA() {
	DEBUG_STR("ekfAttQuat : makeA...");
	double p = u(1) / 2;
	double q = u(2) / 2;
	double r = u(3) / 2;

	A(1, 1) = 1;
	A(1, 2) = -p;
	A(1, 3) = -q;
	A(1, 4) = -r;

	A(2, 1) = p;
	A(2, 2) = 1;
	A(2, 3) = r;
	A(2, 4) = -q;

	A(3, 1) = q;
	A(3, 2) = -r;
	A(3, 3) = 1;
	A(3, 4) = p;

	A(4, 1) = r;
	A(4, 2) = q;
	A(4, 3) = -p;
	A(4, 4) = 1;

	DEBUG_STR("ekfAttQuat : done");
}

void ekfAttQuat::makeBaseW() {
	DEBUG_STR("ekfAttQuat : makeBaseW...");
	W(1, 1) = 1;
	//	W(1,2) = tt * sp;
	//	W(1,3) = tt * cp;
	W(2, 1) = 0;
	//	W(2,2) = cp;
	//	W(2,3) = -sp;
	DEBUG_STR("ekfAttQuat : done");
}

void ekfAttQuat::makeW() {
	DEBUG_STR("ekfAttQuat : make...");
	/*
	 * [ -q1, -q2, -q3]
	 * [  q0, -q3,  q2]
	 * [  q3,  q0, -q1]
	 * [ -q2,  q1,  q0]
	 */
	W(1, 1) = -x(2);
	W(1, 2) = -x(3);
	W(1, 3) = -x(4);
	W(2, 1) = x(1);
	W(2, 2) = -x(4);
	W(2, 3) = x(3);
	W(3, 1) = x(4);
	W(3, 2) = x(1);
	W(3, 3) = -x(2);
	W(4, 1) = -x(3);
	W(4, 2) = x(2);
	W(4, 3) = x(1);
	DEBUG_STR("ekfAttQuat : done");
}

void ekfAttQuat::makeBaseQ() {
	DEBUG_STR("ekfAttQuat : makeBaseQ...");
	Q(1, 1) = 1.00000 * processVar;
	Q(1, 2) = -0.17227 * processVar;
	Q(1, 3) = 0.11208 * processVar;
	Q(2, 1) = -0.17227 * processVar;
	Q(2, 2) = 1.00000 * processVar;
	Q(2, 3) = -0.25561 * processVar;
	Q(3, 1) = 0.11208 * processVar;
	Q(3, 2) = -0.25561 * processVar;
	Q(3, 3) = 1.00000 * processVar;
	DEBUG_STR("ekfAttQuat : Done");
}

void ekfAttQuat::makeH() {
	DEBUG_STR("ekfAttQuat : makeH...");
	/* jacobian(h, x)
	 * [ -2*g*q2, -2*g*q3, -2*g*q0, -2*g*q1]
	 * [  2*g*q1,  2*g*q0, -2*g*q3, -2*g*q2]
	 * [ -2*g*q0,  2*g*q1,  2*g*q2, -2*g*q3]
	 */
	H(1, 1) = -2 * g * x(3); // -2*g*q2
	H(1, 2) = -2 * g * x(4); // -2*g*q3
	H(1, 3) = -2 * g * x(1); // -2*g*q0
	H(1, 4) = -2 * g * x(2); // -2*g*q1
	H(2, 1) = 2 * g * x(2); //  2*g*q1
	H(2, 2) = 2 * g * x(1); //  2*g*q0
	H(2, 3) = -2 * g * x(4); // -2*g*q3
	H(2, 4) = -2 * g * x(3); // -2*g*q2
	H(3, 1) = -2 * g * x(1); // -2*g*q0
	H(3, 2) = 2 * g * x(2); //  2*g*q1
	H(3, 3) = 2 * g * x(3); //  2*g*q2
	H(3, 4) = -2 * g * x(4); // -2*g*q3
	DEBUG_STR("ekfAttQuat : done...");
}

void ekfAttQuat::makeBaseV() {
	DEBUG_STR("ekfAttQuat : makeBaseV...");
	for (int i = 1; i <= V.nrow(); i++)
		for (int j = 1; j <= V.ncol(); j++)
			V(i, j) = (i == j ? 1.0 : 0.0);
	DEBUG_STR("ekfAttQuat : done...");
}

void ekfAttQuat::makeBaseR() {
	DEBUG_STR("ekfAttQuat : makeBaseR...");
	R(1, 1) = 1.000000 * measVar;
	R(2, 1) = 0.480250 * measVar;
	R(3, 1) = -0.041471 * measVar;
	R(1, 2) = 0.480250 * measVar;
	R(2, 2) = 1.000000 * measVar;
	R(3, 2) = -0.111440 * measVar;
	R(1, 3) = -0.041471 * measVar;
	R(2, 3) = -0.111440 * measVar;
	R(3, 3) = 1.000000 * measVar;
	DEBUG_STR("ekfAttQuat : done...");
}

void ekfAttQuat::makeProcess() { // Implements f(x,u,0)
	DEBUG_STR("ekfAttQuat : makeProcess...");
	Vector x_(x.size());

	/* QUATERNION MULTIPLICATION :
	 * Q = [q0 q1 q2 q3], P = [p0 p1 p2 p3]
	 *         +                               +
	 * 		   | p0*q0 - p1*q1 - p2*q2 - p3*q3 |
	 * Q * P = | p0*q1 + p1*q0 - p2*q3 + p3*q2 |
	 *         | p0*q2 + p1*q3 + p2*q0 - p3*q1 |
	 *         | p0*q3 - p1*q2 + p2*q1 + p3*q0 |
	 *         +                               +
	 */

	double p = u(1) * 0.5;
	double q = u(2) * 0.5;
	double r = u(3) * 0.5;

	x_(1) = x(1) - p * x(2) - q * x(3) - r * x(4);
	x_(2) = x(2) + p * x(1) + q * x(4) - r * x(3);
	x_(3) = x(3) - p * x(4) + q * x(1) + r * x(2);
	x_(4) = x(4) + p * x(3) - q * x(2) + r * x(1);

	// Normalise....
	double len = 0;
	for (int i = 1; i <= 4; i++)
		len += x_(i) * x_(i);
	if (len > 1.0001 || len < 0.9999) {
		len = sqrt(len);
		for (int i = 1; i <= 4; i++)
			x_(i) /= len;
	}

	x.swap(x_);
	DEBUG_STR("ekfAttQuat : done");
}

void ekfAttQuat::makeMeasure() { // Implements h(x,0)
	DEBUG_STR("ekfAttQuat : makeMeasure...");
	Vector z_(z.size());

	/* ROTATION OF A VECTOR BY A UNIT QUATERION:
	 * +    +   +                                                   +   +    +
	 * | va'|   |  1-2*q2²-2*q3²   2*q0*q3+2*q1*q2  2*q1*q3-2*q0*q2 |   | va |
	 * | vb'| = | 2*q1*q2-2*q0*q3    1-2*q1²-2*q3²  2*q0*q1+2*q2*q3 | * | vb |
	 * | vc'|   | 2*q0*q2+2*q1*q3  2*q2*q3-2*q0*q1    1-2q1²-2*q2²  |   | vc |
	 * +    +   +                                                   +   +    +
	 *
	 *                   g*(2*q0*q2 - 2*q1*q3)
	 * R * [0;0;-g] =   -g*(2*q0*q1 + 2*q2*q3)
	 *					 g*(2*q1² + 2*q2² - 1)
	 */

	/* Gravitational vector [m/s²]*/
	z_(1) = -g * 2 * (x(1) * x(3) - x(2) * x(4));
	z_(2) =  g * 2 * (x(1) * x(2) + x(3) * x(4));
	z_(3) = -g * (x(1) * x(1) - x(2) * x(2) - x(3) * x(3) + x(4) * x(4));

	/* Centripetal forces [m/s] * [rad/s] = [m/s²]*/
	z_(1) += Va * 0;
	z_(2) += Va * wz;
	z_(3) -= Va * wy;
	z.swap(z_);
	DEBUG_STR("ekfAttQuat : done");
}

void ekfAttQuat::updateAngVel(double newWx, double newWy, double newWz) {
	DEBUG_STR("ekfAttQuat : updateAngvel...");
	wx = newWx;
	wy = newWy;
	wz = newWz;
	DEBUG_STR("ekfAttQuat : Done");
}
void ekfAttQuat::updateAirspeed(double newAirspeed) {
	DEBUG_STR("ekfAttQuat : updateAirspeed");
	Va = newAirspeed;
	DEBUG_STR("ekfAttQuat : done");
}

const ekfAttQuat::Vector& ekfAttQuat::getX(void) {
	Vector xOut(2);
	xOut(1) = atan2(2*(x(1)*x(2) + x(3)*x(4)), 1 - 2*(x(2)*x(2) + x(3)*x(3)));
	xOut(2) = asin(2*(x(1)*x(3) - x(4)*x(2)));
	return xOut;
}

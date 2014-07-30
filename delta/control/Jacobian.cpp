#include "Jacobian.hpp"
#include "Kinematic.hpp"
#include <cmath>

using namespace eeduro::delta;
using namespace eeros::math;

Jacobian::Jacobian() { }

Jacobian::~Jacobian() { }

Vector3 Jacobian::getCartesianVelo(const Vector3& q, const Vector3& tcp, const Vector3& jointvelo) {
	calculate(q,tcp);
	return jacobi * jointvelo;
}

Vector3 Jacobian::getJointVelo(const Vector3& q, const Vector3& tcp, const Vector3& cartesianvelo) {
	calculate(q,tcp);
	return !jacobi * cartesianvelo;
}

Vector3 Jacobian::getDrivetorque(const Vector3& q, const Vector3& tcp, const Vector3& F_tcp) {
	calculate(q,tcp);
	return jacobi.transpose() * F_tcp;
}

void Jacobian::calculate(const Vector3& q, const Vector3& tcp) {
	Vector3 temp1;
	temp1(1) = Kinematic::r+Kinematic::length_A*cos(q(1));
	temp1(2) = 0;
	temp1(3) = Kinematic::length_A*sin(q(1));
	
	Vector3 temp2;
	temp2(1) = Kinematic::r+Kinematic::length_A*cos(q(2));
	temp2(2) = 0;
	temp2(3) = Kinematic::length_A*sin(q(2));
	
	Vector3 temp3;
	temp3(1) = Kinematic::r+Kinematic::length_A*cos(q(3));
	temp3(2) = 0;
	temp3(3) = Kinematic::length_A*sin(q(3));
	
	Vector3 s1 = tcp - Kinematic::rotz1*temp1;
	Vector3 s2 = tcp - Kinematic::rotz2*temp2;
	Vector3 s3 = tcp - Kinematic::rotz3*temp3;
	
	Vector3 temp2_1;
	temp2_1(1) = -Kinematic::length_A*sin(q(1));
	temp2_1(2) = 0;
	temp2_1(3) = Kinematic::length_A*cos(q(1));
	
	Vector3 temp2_2;
	temp2_2(1) = -Kinematic::length_A*sin(q(2));
	temp2_2(2) = 0;
	temp2_2(3) = Kinematic::length_A*cos(q(2));
	
	Vector3 temp2_3;
	temp2_3(1) = -Kinematic::length_A*sin(q(3));
	temp2_3(2) = 0;
	temp2_3(3) = Kinematic::length_A*cos(q(3));
	
	Vector3 b1 = Kinematic::rotz1*temp2_1;
	Vector3 b2 = Kinematic::rotz2*temp2_2;
	Vector3 b3 = Kinematic::rotz3*temp2_3;
	
	Matrix<3,3> tempA;
	tempA(1,1) = s1.transpose()*b1;
	tempA(2,1) = 0;
	tempA(3,1) = 0;
	
	tempA(1,2) = 0;
	tempA(2,2) = s2.transpose()*b2;
	tempA(3,2) = 0;
	
	tempA(1,3) = 0;
	tempA(2,3) = 0;
	tempA(3,3) = s3.transpose()*b3;
	
	Matrix<3,3> tempS;
	tempS(1,1) = s1(1);
	tempS(2,1) = s2(1);
	tempS(3,1) = s3(1);
	
	tempS(1,2) = s1(2);
	tempS(2,2) = s2(2);
	tempS(3,2) = s3(2);
	
	tempS(1,3) = s1(3);
	tempS(2,3) = s2(3);
	tempS(3,3) = s3(3);
	
	Matrix<3,3> tempSi;   // inverse tempS
	
	tempSi = !tempS;
//	if (!tempSi.invert(tempS)) {
//		// crash
//	}

	jacobi = tempSi*tempA;

}

Matrix<3,3> Jacobian::getJacobian() {
	return jacobi;
}

// not yet fully
//Matrix<3,3> Jacobian::jacobi_D(const Vector3& tcp_velocity, Vector3& q)
//{
//	Matrix<3,3> jacobi_D;
//
//	Vector3 temp2_1;
//	temp2_1(1) = -length_A*sin(q(1));
//	temp2_1(2) = 0;
//	temp2_1(3) = length_A*cos(q(1));
//
//	Vector3 temp2_2;
//	temp2_2(1) = -length_A*sin(q(2));
//	temp2_2(2) = 0;
//	temp2_2(3) = length_A*cos(q(2));
//
//	Vector3 temp2_3;
//	temp2_3(1) = -length_A*sin(q(3));
//	temp2_3(2) = 0;
//	temp2_3(3) = length_A*cos(q(3));
//
//
//	Vector3 b1 = rotz1*temp2_1;
//	Vector3 b2 = rotz2*temp2_2;
//	Vector3 b3 = rotz3*temp2_3;
//
//	Vector3 s1d =  tcp_velocity - b1;
//	Vector3 s2d =  tcp_velocity - b2;
//	Vector3 s3d =  tcp_velocity - b3;
//
//
//	Vector3 temp1;
//	temp1(1) = -length_A*cos(q(1));
//	temp1(2) = 0;
//	temp1(3) = -length_A*sin(q(1));
//
//	Vector3 temp2;
//	temp2(1) = -length_A*cos(q(2));
//	temp2(2) = 0;
//	temp2(3) = -length_A*sin(q(2));
//
//	Vector3 temp3;
//	temp3(1) = -length_A*cos(q(3));
//	temp3(2) = 0;
//	temp3(3) = -length_A*sin(q(3));
//
//
//	Vector3 b1d = rotz1*temp1;
//	Vector3 b2d = rotz2*temp2;
//	Vector3 b3d = rotz3*temp3;
//
//
//	math::Matrix<3,3> tempSd;
//	tempSd(1,1) = s1d(1);
//	tempSd(2,1) = s2d(1);
//	tempSd(3,1) = s3d(1);
//
//	tempSd(1,2) = s1d(2);
//	tempSd(2,2) = s2d(2);
//	tempSd(3,2) = s3d(2);
//
//	tempSd(1,3) = s1d(3);
//	tempSd(2,3) = s2d(3);
//	tempSd(3,3) = s3d(3);
//
//
//	math::Matrix<3,3> tempB;
//	tempB(1,1) = s1d.transpose()*b1+;
//	tempB(2,1) = 0;
//	tempB(3,1) = 0;
//
//	tempB(1,2) = 0;
//	tempB(2,2) = s2.transpose()*b2;
//	tempB(3,2) = 0;
//
//	tempB(1,3) = 0;
//	tempB(2,3) = 0;
//	tempB(3,3) = s3.transpose()*b3;
//
//	return jacobi_D;
//}




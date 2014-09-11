#include "Inertia.hpp"
#include "constants.hpp"
using namespace eeduro::delta;
using namespace eeros::math;
using namespace eeros::control;


Inertia::Inertia(Jacobian &jacobi) : jacobi(jacobi) {
	tcpMass = 0;
	for (int i = 0; i < 3; i++)
		tcpMass(i,i) = mtcp;
			
	motorInertia = 0;
	for (int i = 0; i < 3; i++)
		motorInertia(i,i) = jred;
}

void Inertia::run() {
	Vector3 q012, xyz;
	
	q012 = jointPosIn.getSignal().getValue().getSubMatrix<3,1>(0, 0);
	xyz = tcpPosIn.getSignal().getValue().getSubMatrix<3,1>(0, 0);
	
	if (jacobi.calculate(q012, xyz)) {
		Matrix<3,3> jacobian = jacobi.getJacobian();
		if (jacobian.isInvertible()) {
			Matrix<3,3> inverseJacobian = !jacobian;
			Matrix<3,3> inverseJacobianTransposed = inverseJacobian.transpose();

			Matrix<3,3> M = tcpMass + inverseJacobianTransposed * motorInertia * inverseJacobian;
			Vector3 F = M * accelerationIn.getSignal().getValue().getSubMatrix<3,1>(0, 0);
			
			// Gravitation kompensieren: konstante Kraft nach oben
// 			F[2] = 9.81 * mtcp;
			
			// max. Kraft in x-Richtung ist 1 N
// 			double Fx = F[0];
// 			if (F[0] < 0) Fx = -Fx;
// 			
// 			if (Fx > 1)
// 				Fx = 1;
// 			
// 			F[0] = (F[0] < 0) ? -Fx : Fx;
			
			// Feder:
// 			F[2] = 0;
// 			double z_min = -0.04;
// 			if (xyz[2] < z_min)
// 				F[2] = 500 * (z_min - xyz[2]);

			forceOut.getSignal().setValue(Vector4(F[0], F[1], F[2], jred * accelerationIn.getSignal().getValue()[3]));
			forceOut.getSignal().setTimestamp(accelerationIn.getSignal().getTimestamp());
			
			return;
		}
	}
	
	forceOut.getSignal().setValue(0);
	forceOut.getSignal().setTimestamp(accelerationIn.getSignal().getTimestamp());
}

Input<AxisVector>& Inertia::getAccelerationInput() {
	return accelerationIn;
}

Input<AxisVector>& Inertia::getTcpPosInput() {
	return tcpPosIn;
}

Input<AxisVector>& Inertia::getJointPosInput() {
	return jointPosIn;
}

Output<AxisVector>& Inertia::getOut() {
	return forceOut;
}
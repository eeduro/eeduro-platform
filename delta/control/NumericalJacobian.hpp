#ifndef CH_NTB_EEDURO_DELTA_CONTROL_NUMERICALJACOBIAN_HPP
#define CH_NTB_EEDURO_DELTA_CONTROL_NUMERICALJACOBIAN_HPP

#include <eeros/math/Matrix.hpp>

#include "Kinematics.hpp"

namespace eeduro {
	namespace delta {

		template < int M, int N >
		class NumericalJacobian {
		public:
			NumericalJacobian(Kinematics<M,N> &kinematics) : kinematics(kinematics) { }

			virtual bool calculate(const eeros::math::Matrix<N>& q) {
				eeros::math::Matrix<M> tcp;
				if (!kinematics.forward(q, tcp))
					return false;
				
				eeros::math::Matrix<M,N> P;
				eeros::math::Matrix<N,N> Q;
				
				for (int i = 0; i < N; i++) {
					eeros::math::Matrix<N> q_new(q);
					q_new[i] += 0.1;
					
					eeros::math::Matrix<M> tcp_new;
					if (!kinematics.forward(q_new, tcp_new))
						return false;
					
					eeros::math::Matrix<N> dq;
					eeros::math::Matrix<M> dtcp;
					
					dq = (q_new - q);
					dtcp = (tcp_new - tcp);
					
					for (int j = 0; j < M; j++) P(j,i) = dtcp(j);
					for (int j = 0; j < N; j++) Q(j,i) = dq(j);
				}
				
// 				if (!Q.isInvertible())
// 					return false;
				
				jacobian = P * !Q;
				return true;
			}

			virtual const eeros::math::Matrix<M,N>& getJacobian() { return jacobian; }

		private:
			eeros::math::Matrix<M,N> jacobian;
			Kinematics<M,N> &kinematics;
		};
	}
}

#endif /* CH_NTB_EEDURO_DELTA_CONTROL_NUMERICALJACOBIAN_HPP */


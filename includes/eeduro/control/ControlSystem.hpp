#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeduro/eeduro.hpp>
#include <eeduro/hal/Board.hpp>

#include <eeros/control/Sum.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Switch.hpp>

namespace eeduro {
	
	class ControlSystem {
	
	public:
		ControlSystem(double ts);
		
		void start();
		void stop();
		
		eeros::math::Matrix<nofAxis, nofAxis> kM;
		
		eeduro::Board									board;
		eeros::control::Constant<AxisVector>			setpointPos;
		eeros::control::Constant<AxisVector>			setpointSpeed;
		eeros::control::D<AxisVector>					diffEncPos;
		eeros::control::Sum<2, AxisVector>				sumPos;
		eeros::control::Gain<AxisVector>				posController;
		eeros::control::D<AxisVector>					diffSetPointPos;
		eeros::control::Sum<3, AxisVector>				sumSpeed;
		eeros::control::Gain<AxisVector>				speedController;
		eeros::control::Switch<2, AxisVector>			switchSpeed;
		eeros::control::Gain<AxisVector>				inertia;
		eeros::control::Gain<AxisVector>				invMotConst;
		
	private:
		eeros::control::TimeDomain timedomain;
	};
}

#endif // CONTROLSYSTEM_HPP_

#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeduro/Board.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/Saturation.hpp>
#include "DirectKinematic.hpp"
#include "Jacobi.hpp"
#include "MotorModel.hpp"
#include "constants.hpp"
#include "types.hpp"

namespace eeduro {
	namespace delta {
		
		class ControlSystem {
			
		public:
			ControlSystem();
			
			void start();
			void stop();
			
			// Methods for the sequencer
			void resetEncoders();
			void enableAxis();
			void disableAxis();
			void initBoard();
			bool initAxis();
			void switchToPosControl();
			AxisVector getCurrentPos();
			AxisVector getCurrentAxisPos();
			void goToPos(double x, double y, double z, double phi);
			
		protected:
			AxisVector i;
			Kinematic kinematic;
			
			// Blocks
			eeduro::Board										board;
			
			eeros::control::Constant<AxisVector>				posSetPoint;
			eeros::control::Sum<2, AxisVector>					posSum;
			eeros::control::Gain<AxisVector>					posController;
			eeros::control::D<AxisVector>						posDiff;
			
			eeros::control::Sum<3, AxisVector>					speedSum;
			eeros::control::Constant<AxisVector>				speedSetPoint;
			eeros::control::Saturation<AxisVector>				speedLimitation;
			eeros::control::Gain<AxisVector>					speedController;
			
			eeros::control::Gain<AxisVector, AxisSquareMatrix>	inertia;
			eeros::control::Saturation<AxisVector>				forceLimitation;
			eeduro::delta::Jacobi								jacobi;
			
			eeros::control::Saturation<AxisVector>				torqueLimitation;
			eeros::control::Gain<AxisVector, AxisVector, true>	torqueGear;
			eeros::control::Gain<AxisVector, AxisVector, true>	angleGear;
			eeduro::delta::MotorModel<AxisVector>				motorModel;
			eeros::control::D<AxisVector>						angleDiff;
			eeduro::delta::DirectKinematic						directKin;
			
			eeros::control::TimeDomain timedomain;
		};
	}
}

#endif // CONTROLSYSTEM_HPP_

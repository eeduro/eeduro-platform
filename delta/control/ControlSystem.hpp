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
#include <eeros/control/XBoxInput.hpp>
#include "Kinematic.hpp"
#include "Jacobian.hpp"
#include "DirectKinematic.hpp"
#include "Jacobi.hpp"
#include "MotorModel.hpp"
#include "PathPlanner.hpp"
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
			void enableAxis();
			void disableAxis();
			void initBoard();
			void setVoltageForInitializing(AxisVector u);
			bool switchToPosControl();
			bool axisHomed();
			AxisVector getTcpPos();
			AxisVector getAxisPos();
			void goToPos(double x, double y, double z, double phi);
			
//		protected:
			AxisVector i;
			AxisVector kM;
			AxisVector RA;
			Kinematic kinematic;
			Jacobian jacobian;
			bool homed;
			
			// Blocks			
			eeduro::Board										board;
			
			eeros::control::XBoxInput							joystick;
			
// 			eeros::control::Constant<AxisVector>				posSetPoint;
// 			eeduro::delta::PathPlanner							pathPlanner;
			eeros::control::Sum<2, AxisVector>					posSum;
			eeros::control::Gain<AxisVector>					posController;
			eeros::control::D<AxisVector>						posDiff;
			
			eeros::control::Sum<3, AxisVector>					speedSum;
// 			eeros::control::Constant<AxisVector>				speedSetPoint;
			eeros::control::Saturation<AxisVector>				speedLimitation;
			eeros::control::Gain<AxisVector>					speedController;
			eeros::control::Sum<2, AxisVector>					accSum;
			
			eeros::control::Gain<AxisVector>					inertia;
			eeros::control::Saturation<AxisVector>				forceLimitation;
			eeduro::delta::Jacobi								jacobi;
			
			eeros::control::Saturation<AxisVector>				torqueLimitation;
			eeros::control::Gain<AxisVector, AxisVector, true>	torqueGear;
			eeros::control::Gain<AxisVector, AxisVector, true>	angleGear;
			eeduro::delta::MotorModel							motorModel;
			eeros::control::Switch<2, AxisVector>				voltageSwitch;
			eeros::control::Constant<AxisVector>				voltageSetPoint;
			eeduro::delta::DirectKinematic						directKin;
			
			eeros::control::TimeDomain timedomain;
			
		private:
			bool allAxisStopped(double maxSpeed = 0.001);
		};
	}
}

#endif // CONTROLSYSTEM_HPP_

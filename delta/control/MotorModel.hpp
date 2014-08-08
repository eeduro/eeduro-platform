#ifndef CH_NTB_EEDURO_DELTA_MOTORMODEL_HPP
#define CH_NTB_EEDURO_DELTA_MOTORMODEL_HPP

#include <eeros/control/Block1i1o.hpp>
#include "types.hpp"

namespace eeduro {
	namespace delta {
		class MotorModel : public eeros::control::Block {
		public:
			MotorModel(const AxisVector kM, const AxisVector RA);
			
			virtual eeros::control::Input<AxisVector>& getTorqueIn();
			virtual eeros::control::Input<AxisVector>& getSpeedIn();
			virtual eeros::control::Output<AxisVector>& getOut();
			
			virtual void run();
			
		protected:
			eeros::control::Input<AxisVector> torque;
			eeros::control::Input<AxisVector> speed;
			eeros::control::Output<AxisVector> voltage;
			
			AxisVector kM, RA;
		};
	}
}

#endif /* CH_NTB_EEDURO_DELTA_MOTORMODEL_HPP */

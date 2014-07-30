#ifndef CH_NTB_EEDURO_DELTA_MOTORMODEL_HPP
#define CH_NTB_EEDURO_DELTA_MOTORMODEL_HPP

#include <eeros/control/Block1i1o.hpp>
#include "types.hpp"

namespace eeduro {
	namespace delta {
		template <typename T = double>
		class MotorModel : public eeros::control::Block {
		public:
			MotorModel() { }
			
			virtual eeros::control::Input<T>& getTorqueIn() {
				return torque;
			}
			
			virtual eeros::control::Input<T>& getSpeedIn() {
				return speed;
			}
			
			virtual eeros::control::Output<T>& getOut() {
				return voltage;
			}
			
			virtual void run() {
				// TODO
			}
			
		protected:
			eeros::control::Input<T> torque;
			eeros::control::Input<T> speed;
			eeros::control::Output<T> voltage;
			
		};
	}
}

#endif /* CH_NTB_EEDURO_DELTA_MOTORMODEL_HPP */

#ifndef CH_NTB_EEDURO_DELTA_JACOBI_HPP
#define CH_NTB_EEDURO_DELTA_JACOBI_HPP

#include <eeros/control/Block1i1o.hpp>
#include "types.hpp"

namespace eeduro {
	namespace delta {
		class Jacobi : public eeros::control::Block1i1o<AxisVector> {
		public:
			Jacobi();
			virtual ~Jacobi();
			
			virtual void run();
			
		private:
			
		};
	}
}

#endif /* CH_NTB_EEDURO_DELTA_JACOBI_HPP */

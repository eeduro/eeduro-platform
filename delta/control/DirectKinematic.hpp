#ifndef CH_NTB_EEDURO_DELTA_DIRECTKINEMATIC_HPP
#define CH_NTB_EEDURO_DELTA_DIRECTKINEMATIC_HPP

#include <eeros/control/Block1i1o.hpp>
#include "types.hpp"

namespace eeduro {
	namespace delta {
		class DirectKinematic : public eeros::control::Block1i1o<AxisVector> {
		public:
			DirectKinematic();
			virtual ~DirectKinematic();
			
			virtual void run();
			
		private:
			
		};
	}
}

#endif /* CH_NTB_EEDURO_DELTA_DIRECTKINEMATIC_HPP */

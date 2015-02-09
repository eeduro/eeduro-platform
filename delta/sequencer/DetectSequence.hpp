#ifndef CH_NTB_EEDURO_DEMO_DETECTSEQUENCE_HPP_
#define CH_NTB_EEDURO_DEMO_DETECTSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include "../control/ControlSystem.hpp"
#include "../Calibration.hpp"

namespace eeduro {
	namespace delta {
		class DetectSequence : public eeros::sequencer::Sequence<int, int> {
		public:
			DetectSequence(eeros::sequencer::Sequencer* sequencer, eeduro::delta::ControlSystem* controlSys);
			
			virtual int run(int position);
			
		private:
			virtual void waitUntilPointReached();
			
			eeduro::delta::ControlSystem* controlSys;
			Calibration calibration;
		};
	}
}

#endif // CH_NTB_EEDURO_DEMO_DETECTSEQUENCE_HPP_

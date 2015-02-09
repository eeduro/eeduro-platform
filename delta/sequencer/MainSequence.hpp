#ifndef CH_NTB_EEDURO_DELTA_MAINSEQUENCE_HPP_
#define CH_NTB_EEDURO_DELTA_MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/ControlSystem.hpp"
#include "MoveBlockSequence.hpp"
#include "SortSequence.hpp"

namespace eeduro {
	namespace delta {
		class MainSequence : public eeros::sequencer::Sequence<void> {
		public:
			MainSequence(eeros::sequencer::Sequencer* sequencer, eeduro::delta::ControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
			
			virtual void run();
			
		private:
			MoveBlockSequence moveBlock;
			SortSequence sort;
			
			eeduro::delta::ControlSystem* controlSys;
			eeros::safety::SafetySystem* safetySys;
		};
	}
}

#endif // CH_NTB_EEDURO_DELTA_MAINSEQUENCE_HPP_

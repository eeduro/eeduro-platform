#ifndef CH_NTB_EEDURO_DELTA_SORTSEQUENCE_HPP_
#define CH_NTB_EEDURO_DELTA_SORTSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/ControlSystem.hpp"
#include "MoveBlockSequence.hpp"
#include "DetectSequence.hpp"
#include <array>

namespace eeduro {
	namespace delta {
		class SortSequence : public eeros::sequencer::Sequence<void> {
		public:
			SortSequence(eeros::sequencer::Sequencer* sequencer, eeduro::delta::ControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
			
			virtual void run();
			
		private:
			virtual void move(int position);
			virtual void waitUntilPointReached();
			virtual void sortBlocks(std::array<int,4> blocks);
			virtual int find(const std::array<int,4> &blocks, int block);
			
			MoveBlockSequence moveBlock;
			DetectSequence detect;
			eeduro::delta::ControlSystem* controlSys;
			eeros::safety::SafetySystem* safetySys;
			Calibration calibration;
		};
	}
}

#endif // CH_NTB_EEDURO_DELTA_SORTSEQUENCE_HPP_

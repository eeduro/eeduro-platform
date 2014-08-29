#ifndef CH_NTB_EEDURO_DEMO_MOVEBLOCKSEQUENCE_HPP_
#define CH_NTB_EEDURO_DEMO_MOVEBLOCKSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/ControlSystem.hpp"

namespace eeduro {
	namespace delta {
		class MoveBlockSequence : public eeros::sequencer::Sequence<void, int, int> {
		public:
			MoveBlockSequence(eeros::sequencer::Sequencer* sequencer, eeduro::delta::ControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
			
			virtual void run(int from, int to);
			virtual void run(int to);
			
			virtual bool checkPreCondition();
			
			virtual void init();
			virtual void exit();
			
		private:
			virtual void up();
			virtual void down();
			virtual void grab();
			virtual void release();
			virtual void move(int position);
			
			eeduro::delta::ControlSystem* controlSys;
			eeros::safety::SafetySystem* safetySys;
			
			int state;
			int from;
			int to;
		};
	}
}

#endif // CH_NTB_EEDURO_DEMO_MOVEBLOCKSEQUENCE_HPP_

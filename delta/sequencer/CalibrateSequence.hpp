#ifndef CH_NTB_EEDURO_DELTA_CALIBRATESEQUENCE_HPP_
#define CH_NTB_EEDURO_DELTA_CALIBRATESEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/ControlSystem.hpp"
#include "../Calibration.hpp"
#include <vector>

namespace eeduro {
	namespace delta {
		class CalibrateSequence : public eeros::sequencer::Sequence<void> {
		public:
			CalibrateSequence(eeros::sequencer::Sequencer* sequencer, eeduro::delta::ControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
			
			virtual void run();
			
		protected:
			void waitUntilReady();
			void waitForButton(std::vector<int> buttons);
			void waitForGreenButton();
			void waitForRedButton();
			void waitForBlueButton();
			void waitForBlueOrRedButton();
			
		private:
			Calibration calibration;
			eeduro::delta::ControlSystem* controlSys;
			eeros::safety::SafetySystem* safetySys;
		};
	}
}

#endif // CH_NTB_EEDURO_DELTA_CALIBRATESEQUENCE_HPP_

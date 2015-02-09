#include "CalibrateSequence.hpp"
#include "../safety/DeltaSafetyProperties.hpp"
#include <eeros/core/EEROSException.hpp>
#include <unistd.h>

using namespace eeduro::delta;
using namespace eeros::sequencer;
using namespace eeros::safety;


CalibrateSequence::CalibrateSequence(Sequencer* sequencer, ControlSystem* controlSys, SafetySystem* safetySys) :
	Sequence<void>("main", sequencer),
	controlSys(controlSys),
	safetySys(safetySys)
{
	
}

void CalibrateSequence::waitUntilReady() {
	while(safetySys->getCurrentLevel().getId() != systemReady) {
		usleep(100000);
		yield();
	}
}

void CalibrateSequence::waitForButton(std::vector<int> buttons) {
	for (auto i: buttons) {
		if (i < 0 || i > 3)
			throw EEROSException("index out of range");
	}
	usleep(200000);
	for (auto i: buttons) controlSys->board.button_latch[i].reset();
	while(true) {
		for (auto i: buttons) {
			if (controlSys->board.button_latch[i].get())
				return;
		}
		usleep(100000);
		yield();
	}
}

void CalibrateSequence::waitForGreenButton()	 { waitForButton({2}); }
void CalibrateSequence::waitForRedButton()		 { waitForButton({1}); }
void CalibrateSequence::waitForBlueButton()		 { waitForButton({0}); }
void CalibrateSequence::waitForBlueOrRedButton() { waitForButton({ 0, 1 }); }

void CalibrateSequence::run() {
	waitUntilReady();
	
	usleep(500000);
	
	//safetySys->triggerEvent(events::doEmergency);
	controlSys->disableAxis();
	calibration.loadDefaults();
	
	const char *block[] = { "[no block]", "[block 1]", "[block 2]", "[block 3]" };

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			log.trace() << block[i] << " move TCP to position " << j << " and press the blue button";
			waitForBlueButton();
			
			auto p = controlSys->directKin.getOut().getSignal().getValue();
			calibration.position[j].zblock[i] = p[2];
			if (i == 3) {
				calibration.position[j].x = p[0];
				calibration.position[j].y = p[1];
			}
		}
	}

	for (int i = 0; i < 4; i++) {
		calibration.position[i].level12 = (calibration.position[i].zblock[1] + calibration.position[i].zblock[2]) / 2.0;
		calibration.position[i].level23 = (calibration.position[i].zblock[2] + calibration.position[i].zblock[3]) / 2.0;
		calibration.position[i].level30 = (calibration.position[i].zblock[3] + calibration.position[i].zblock[0]) / 2.0;
	}
	if (calibration.save()) {
		log.info() << "calibration saved";
	}
	else {
		log.error() << "calibration could not be saved";
	}
}

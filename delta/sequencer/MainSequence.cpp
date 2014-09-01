#include "MainSequence.hpp"
#include "../safety/DeltaSafetyProperties.hpp"
#include <unistd.h>

using namespace eeduro::delta;
using namespace eeros::sequencer;
using namespace eeros::safety;

enum {
	idle,
	move_from_to,
	move_to
};

MainSequence::MainSequence(Sequencer* sequencer, ControlSystem* controlSys, SafetySystem* safetySys) : Sequence<void>("main", sequencer), controlSys(controlSys), safetySys(safetySys), moveBlock(sequencer, controlSys, safetySys) {
	// nothing to do
}

void MainSequence::run() {
	log.trace() << "Sequencer '" << name << "': started.";
	yield();
	log.trace() << "Sequencer '" << name << "': waiting until robot is ready...";
	while(safetySys->getCurrentLevel().getId() != systemReady) {
		yield();
	}

	log.info() << "Press the blue button for automatic play.";
	while(controlSys->board.button_latch[0].get() != true) {
		usleep(100000);
		yield();
	}
	
	
	log.trace() << "Sequencer '" << name << "': started.";
}



#include "control/ControlSystem.hpp"
#include "safety/DeltaSafetyProperties.hpp"
#include "sequencer/MoveBlockSequence.hpp"
#include "sequencer/MainSequence.hpp"

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/logger/SysLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>

#include <iostream>
#include <unistd.h>
#include <signal.h>

using namespace eeduro;
using namespace eeduro::delta;
using namespace eeros;
using namespace eeros::logger;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::sequencer;

volatile bool running = true;

void signalHandler(int signum) {
	running = false;
}

int main(int argc, char* argv[]) {
	signal(SIGINT, signalHandler);
	
	StreamLogWriter w(std::cout);
	SysLogWriter s("delta");
	w.show();
	s.show();
	
	Logger<LogWriter>::setDefaultWriter(&w);
// 	Logger<LogWriter>::setDefaultWriter(&s);
	
	Logger<LogWriter> log('M');
	
	log.trace() << "Application eeduro-delta started...";
	
	// create control system
	ControlSystem controlSys;
	
	// initialize hardware
	controlSys.initBoard();
	
	controlSys.mouse.j.on_button([&](int x, bool value) {
		if (x == BTN_LEFT || x == BTN_RIGHT)
			controlSys.board.power_out[0] = value;
	});
	
	// create safety system
	DeltaSafetyProperties safetyProperties(&controlSys);
	SafetySystem safetySys(safetyProperties, dt);
	
	// create sequencer
	Sequencer sequencer;
	MainSequence mainSequence(&sequencer, &controlSys, &safetySys);
	sequencer.start(&mainSequence);
	
	while(running && sequencer.getState() != state::terminated) {
		usleep(1000000);
	}
	
	log.info() << "Shuting down...";
	
	safetySys.triggerEvent(doParking);
	safetySys.shutdown();
	
	controlSys.disableAxis();
	controlSys.stop();
	
	return 0;
}

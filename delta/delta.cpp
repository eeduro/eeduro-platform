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
	
	// create safety system
	DeltaSafetyProperties safetyProperties(&controlSys);
	SafetySystem safetySys(safetyProperties, dt);
	
	// create sequencer
	Sequencer sequencer;
	MainSequence mainSequence(&sequencer, &controlSys, &safetySys);
	sequencer.start(&mainSequence);
	
	// 	constexpr int nofpos = 6;
// 	double x[nofpos] = {0.01, 0.01, -0.01, -0.01, 0.0, 0.0};
// 	double y[nofpos] = {-0.01, 0.01, 0.01, -0.01, 0.0, 0.0};
// 	double z[nofpos] = {-0.01, -0.02, -0.01, -0.05, -0.06, -0.05};
// 	double phi[nofpos] = {0.1, 0.5, 0.1, 0.5, 0.1, 0.5};
// 	int i = 0;
// 	AxisVector limit = {0, 100, 100, 100};
// 	controlSys.forceLimitation.setLimit(-limit, limit);
	while(running && sequencer.getState() != state::terminated) {
// 		std::cout << "TCP z: " << controlSys.pathPlanner.getPosOut().getSignal().getValue()[2] << std::endl;
//		std::cout << controlSys.joystick.getOut().getSignal().getValue() << std::endl;
// 		std::cout << controlSys.inputSwitch.getOut().getSignal().getValue() << std::endl;
		std::cout << controlSys.inputSwitch.getOut().getSignal().getValue() << "  |  ";
		std::cout << controlSys.directKin.getOut().getSignal().getValue() << "  |  ";
		std::cout << controlSys.angleGear.getOut().getSignal().getValue() << std::endl;
// 		if(controlSys.axisHomed()) {
// 			controlSys.goToPos(0, 0, z[i], 0.2);
// 			controlSys.forceLimitation.enable();
// 			controlSys.goToPos(0, 0, -0.03, 0.5);
// 			i = (i + 1) % nofpos;
// 		}
		usleep(1000000);
	}
	
	log.info() << "Shuting down...";
	
	safetySys.triggerEvent(doParking);
	safetySys.shutdown();
	
	controlSys.disableAxis();
	controlSys.stop();
	
	return 0;
}

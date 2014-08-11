#include "control/ControlSystem.hpp"

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

#include <iostream>
#include <unistd.h>

using namespace eeduro;
using namespace eeduro::delta;
using namespace eeros;
using namespace eeros::logger;

int main(int argc, char* argv[]) {
	std::cout << "Application eeduro-delta started..." << std::endl;
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show();
	
	// create control system
	ControlSystem cs;
	
	// initialize hardware
	cs.initBoard();
		
	// start control system
	cs.start();
	
	cs.temp.setValue({0.1, 0, 0, 0});
	
	// initialize axis
	std::cout << "  Initializing axis... " << std::endl;
	cs.enableAxis();
	cs.initAxis();
//	cs.disableAxis();
	std::cout << "  -> done" << std::endl;
	
	for(int i = 0; i < 10; i++) {
		std::cout << cs.torqueGear.getOut().getSignal().getValue() << " -> " << cs.voltageSwitch.getOut().getSignal().getValue() << std::endl;
		sleep(1);
	}
	
	cs.disableAxis();
	
	// stop control system
	cs.stop();
	
	return 0;
}

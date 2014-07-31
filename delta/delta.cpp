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
	
	// initialize axis
//	cs.enableAxis();
	
	cs.resetEncoders();
	
	for(int i = 0; i < 30; i++) {
//		std::cout << "Axis: [q0, q1, q2, q3] = " << cs.getCurrentAxisPos() << std::endl;
		std::cout << "TCP: [x, y, z, phi] = " << cs.getCurrentPos() << std::endl;
		sleep(1);
	}
	
	// stop control system
	cs.stop();
	
	return 0;
}

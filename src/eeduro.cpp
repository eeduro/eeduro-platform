#include <eeduro/eeduro.hpp>
#include <eeduro/control/ControlSystem.hpp>

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/core/EEROSException.hpp>

#include <iostream>
#include <unistd.h>

using namespace eeduro;
using namespace eeros;
using namespace eeros::logger;

int main(int argc, char* argv[]) {
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show();
	
	// create control system
	ControlSystem cs(dt);
	
	// initialize hardware
	if(!cs.board.open("/dev/spidev1.0"))
		throw EEROSException("failed to open SPI device");
	
	// start control system
	cs.start();
	
	// wait for a while
	sleep(5);
	
	// stop control system
	cs.stop();
	
	return 0;
}

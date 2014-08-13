#include "control/ControlSystem.hpp"
#include "safety/DeltaSafetyProperties.hpp"

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

#include <iostream>
#include <unistd.h>
#include <signal.h>

using namespace eeduro;
using namespace eeduro::delta;
using namespace eeros;
using namespace eeros::logger;
using namespace eeros::hal;
using namespace eeros::safety;

volatile bool running = true;

void signalHandler(int signum) {
	running = false;
}

int main(int argc, char* argv[]) {
	signal(SIGINT, signalHandler);
	
	std::cout << "Application eeduro-delta started..." << std::endl;
	
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show();
	Logger<LogWriter> log('M');
	
	// create control system
	ControlSystem controlSys;
	
	// initialize hardware
	controlSys.initBoard();
	
	// create safety system
	DeltaSafetyProperties safetyProperties(&controlSys);
	SafetySystem safetySys(safetyProperties, dt);
	
// 	// initialize axis
// 	std::cout << "  Initializing axis... " << std::endl;
// 	controlSys.enableAxis();
// 	controlSys.initAxis();
// 	std::cout << "  -> done" << std::endl;
// 	
// 	//cs.goToPos(0.0, 0.0, -0.02, 0.1);
// 	controlSys.posSetPoint.setValue({0.0, 0.0, -0.04, -0.01});
// 	
// 	double x = 0, y = 0;
// 	for(int i = 0; i < 20; i++) {
// 		std::cout << controlSys.jacobi.getOut().getSignal().getValue() << " -> " << controlSys.torqueLimitation.getOut().getSignal().getValue() << std::endl;
// 		sleep(1);
// 		controlSys.posSetPoint.setValue({x, y, -0.04, -0.01});
// 		x += 0.01; y += 0.01;
// 		if(x > 0.02) x = -0.02;
// 		if(y > 0.02) y = -0.02;
// 		std::cout << controlSys.board.button[0]  << controlSys.board.button[1] << std::endl;
// 	}
// 	
// 	controlSys.disableAxis();
// 	sleep(1);
// 	
// 	// stop control system
// 	controlSys.stop();
	while(running) {
// 		std::cout << "[E] " << HAL::instance().getLogicPeripheralInput("emergency")->get() << " (" << controlSys.board.button[0] << ')' << std::endl;
// 		std::cout << "[A] " << HAL::instance().getLogicPeripheralInput("approval")->get() << " (" << controlSys.board.button[1] <<  ')' << std::endl;
//		std::cout << "q = " << controlSys.getAxisPos() << std::endl;
		controlSys.board.power_out[0] ^= true;
//		controlSys.board.power_out[1] ^= true;
		usleep(300000);
	}
	
	log.info() << "Shuting down..." << endl;
	
	safetySys.triggerEvent(doParking);
	safetySys.shutdown();
	
	controlSys.disableAxis();
	controlSys.stop();
	
	return 0;
}

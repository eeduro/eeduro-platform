#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <eeduro/Board.hpp>

volatile bool running = true;

void signalHandler(int signum) {
	running = false;
}

int main(int argc, char* argv[])
{
	eeduro::Board board;
	
	signal(SIGINT, signalHandler);
	
	if (!board.open("/dev/spidev1.0")) {
		std::cerr << "cannot open SPI device " << std::endl;
		return 1;
	}

	eeros::math::Vector4 voltage;
	voltage.zero();
	
	eeros::control::Output<eeros::math::Vector4> voltageOutput;
	board.getIn().connect(voltageOutput);
	
	board.setReset(false);
	board.setEnable(true);
	
	while (running) {
		voltage[3] = 3;
		voltageOutput.getSignal().setValue(voltage);
		board.run();
		sleep(1);
		voltage[3] = -3;
		voltageOutput.getSignal().setValue(voltage);
		board.run();
		sleep(1);
	}
	
	voltage.zero();
	voltageOutput.getSignal().setValue(voltage);
	
	board.setReset(true);
	board.setEnable(false);
	
	board.run();
	board.close();
	
	return 0;
}

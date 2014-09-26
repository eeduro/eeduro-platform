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

	while (running) {
		board.power_out[0] = !board.power_out[0];
		board.run();
		usleep(500000);
	}
	board.power_out[0] = false;
	board.run();

	board.close();
	return 0;
}

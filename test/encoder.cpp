#include <iostream>
#include <unistd.h>
#include <eeduro/Board.hpp>

int main(int argc, char *argv[])
{
	eeduro::Board board;
	
	if (!board.open("/dev/spidev1.0")) {
		std::cerr << "cannot open SPI device " << std::endl;
		return 1;
	}

	int counter = 0;
	board.resetPositions();
	while (true) {
		board.run();
		if (counter++ >= 250)
		{
			counter = 0;

			for (int i = 0; i < 4; i++) {
				if (i != 0) std::cout << "    ";
				std::cout << board.axis[i].position;
			}
			std::cout << std::endl;
		}
		usleep(1000);
	}

	board.close();
	return 0;
}

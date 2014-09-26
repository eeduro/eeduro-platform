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

	bool button[3] = { };
	while (true) {
		board.run();
		for (int i = 0; i < 3; i++) {
			if (button[i] != board.button[i]) {
				
				if (board.button[i]) {
					std::cout << "button " << i << ": down" << std::endl;
				} else {
					std::cout << "button " << i << ": up" << std::endl;
				}
				
				button[i] = board.button[i];
			}
		}
		usleep(30000);
	}

	board.close();
	return 0;
}

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

	for (int i = 0; i < 16; i++) {
		for (int j = 0; j < 3; j++) {
			board.led[j] = ( (i & (1 << j)) != 0);
		}
		board.run();
		sleep(1);
	}

	board.close();
	return 0;
}

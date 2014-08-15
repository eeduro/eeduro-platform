#include <cstdlib>
#include <iostream>
#include <getopt.h>

#include <eeduro/Board.hpp>

void help()
{
	using namespace std;
	
	cout << "eeduro-ctrl [-hed ] [ v0 [ v1 [ v2 [ v3 ] ] ] ]" << endl;
	cout << "            [ --button index   | -b index ]" << endl;
	cout << "            [ --position axis  | -p axis  ]" << endl;
	cout << "            [ --led-on index   | -l index ]" << endl;
	cout << "            [ --led-off index  | -L index ]" << endl;
}


int main(int argc, char *argv[])
{
	using namespace std;

	eeduro::Board board;

	const char *device = "/dev/spidev1.0";
	
	if (board.open(device))
	{
		eeros::math::Matrix<4> v;
		v.zero();
		
		board.setReset(true);
		board.setEnable(false);
		board.getIn().getSignal().setValue(v);
		
		static const char* short_options = "-hedb:p:l:L:";
		static struct option long_options[] =
		{
			{ "help",     no_argument,       nullptr, 'h' },
			{ "enable",   no_argument,       nullptr, 'e' },
			{ "disable",  no_argument,       nullptr, 'd' },
			{ "button",   required_argument, nullptr, 'b' },
			{ "position", required_argument, nullptr, 'p' },
			{ "led-on",   required_argument, nullptr, 'l' },
			{ "led-off",  required_argument, nullptr, 'L' },
			{ 0, 0, 0, 0 }
		};
	
		int option_index = 0;
		int axis_index = 0;
		bool run = true;
		while (run)
		{
			int r = getopt_long(argc, argv, short_options, long_options, &option_index);
			switch (r)
			{
				case 'h':
					help();
					board.close();
					return 0;
					
				case 'e':
					board.setReset(false);
					board.setEnable(true);
					break;
					
				case 'd':
					board.setReset(true);
					board.setEnable(false);
					break;
					
				case 'b':
					{
						int button_index = atoi(optarg);
						if (button_index >= 0 && button_index <= 2)
						{
							board.run();
							cout << board.button[button_index] << endl;
						}
						else
						{
							cerr << "button index (" << button_index << ") out of range [0..2]" << endl;
							board.close();
							return 4;
						}
					}
					break;
					
				case 'l':
				case 'L':
					{
						int led_index = atoi(optarg);
						if (led_index >= 0 && led_index <= 3)
						{
							board.led[led_index] = (r == 'l');
						}
						else
						{
							cerr << "LED index (" << led_index << ") out of range [0..3]" << endl;
							board.close();
							return 4;
						}
					}
					break;
					
				case 'p':
					{
						int axis = atoi(optarg);
						if (axis >= 0 && axis <= 3)
						{
							board.run();
							cout << board.axis[axis].position << endl;
						}
						else
						{
							cerr << "axis (" << axis << ") out of range [0..3]" << endl;
							board.close();
							return 5;
						}
					}
					break;
					
				case '?':
					board.close();
					return 2;
					
				case 1:
					if (axis_index < 4)
					{
							board.setReset(false);
							board.setEnable(true);
							v(axis_index++) = atof(optarg);
					}
					else
					{
							cerr << "there are only 4 axis" << endl;
							board.close();
							return 3;
					}
					break;
				
				default:
					run = false;
					break;
			}
		}

		board.run();
		board.close();
		return 0;
	}
	else
	{
		cerr << "cannot open device " << device << endl;
	}

	return 1;
}

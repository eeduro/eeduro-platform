#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <iostream>

using namespace eeros::logger;

int main(int argc, char* argv[]) {
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	w.show();
	
	return 0;
}

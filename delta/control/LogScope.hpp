#ifndef EEDURO_TEXTSCOPE_HPP
#define EEDURO_TEXTSCOPE_HPP

#include <eeros/control/Block1i.hpp>
#include <eeros/core/EEROSException.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/Logger.hpp>

#include <string>

namespace eeduro {

	template <typename T = double>
	class LogScope : public eeros::control::Block1i<T> {

	public:
		LogScope(std::string prefix, eeros::logger::LogWriter* w = nullptr) {
			if(w != nullptr) log.set(w);
		}
		
		virtual void run() {
			log.info() << prefix << this->in.getSignal().getValue() << " (" << this->in.getSignal().getTimestamp() << ")";
		}
		
	private:
		eeros::logger::Logger<eeros::logger::LogWriter> log;
		std::string prefix;
	};
};

#endif // EEDURO_TEXTSCOPE_HPP

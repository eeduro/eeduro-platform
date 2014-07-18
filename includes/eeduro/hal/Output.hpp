#ifndef __CH_NTB_EEDURO_Output_HPP
#define __CH_NTB_EEDURO_Output_HPP

#include <eeros/hal/PeripheralOutput.hpp>

namespace eeduro
{
	template <typename T>
	class Output : public eeros::hal::PeripheralOutput<T>
	{
	public:
		Output(std::string id, T& value) : eeros::hal::PeripheralOutput<T>(id), value(value) { }
		Output(std::string id, T* value) : eeros::hal::PeripheralOutput<T>(id), value(*value) { }
		virtual ~Output() { }
		virtual T get() { return value; }
		virtual void set(T value) { this->value = value; }
	private:
		T& value;
	};
}

#endif /* __CH_NTB_EEDURO_Output_HPP */

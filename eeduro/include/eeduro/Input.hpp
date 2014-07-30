#ifndef __CH_NTB_EEDURO_Input_HPP
#define __CH_NTB_EEDURO_Input_HPP

#include <eeros/hal/PeripheralInput.hpp>

namespace eeduro {
	namespace hal {
		template <typename T>
		class Input : public eeros::hal::PeripheralInput<T>
		{
		public:
			Input(std::string id, T& value) : eeros::hal::PeripheralInput<T>(id), value(value) { }
			Input(std::string id, T* value) : eeros::hal::PeripheralInput<T>(id), value(*value) { }
			virtual ~Input() { }
			virtual T get() { return value; }
		private:
			T& value;
		};
	}
}

#endif /* __CH_NTB_EEDURO_Input_HPP */

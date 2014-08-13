#ifndef __CH_NTB_EEDURO_Input_HPP
#define __CH_NTB_EEDURO_Input_HPP

#include <eeros/hal/PeripheralInput.hpp>

namespace eeduro {
	namespace hal {
		template <typename T>
		class Input : public eeros::hal::PeripheralInput<T> {
		public:
			Input(std::string id, T& value) : eeros::hal::PeripheralInput<T>(id), value(value) { }
			Input(std::string id, T* value) : eeros::hal::PeripheralInput<T>(id), value(*value) { }
			virtual ~Input() { }
			virtual T get() { return value; }
		private:
			T& value;
		};
		
// 		template <>
// 		class Input<bool> : public eeros::hal::PeripheralInput<bool> {
// 		public:
// 			Input(std::string id, bool& value, bool inverted = false) : eeros::hal::PeripheralInput<bool>(id), value(value), inverted(inverted) { }
// 			Input(std::string id, bool* value, bool inverted = false) : eeros::hal::PeripheralInput<bool>(id), value(*value), inverted(inverted) { }
// 			virtual ~Input() { }
// 			virtual bool get() { if(inverted) return !value; else return value; }
// 		private:
// 			bool inverted;
// 			bool& value;
// 		};
	}
}

#endif /* __CH_NTB_EEDURO_Input_HPP */

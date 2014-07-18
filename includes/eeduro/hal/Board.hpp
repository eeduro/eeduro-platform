#ifndef __CH_NTB_EEDURO_BOARD_HPP_
#define __CH_NTB_EEDURO_BOARD_HPP_

#include <eeduro/hal/Latch.hpp>
#include <eeduro/hal/Input.hpp>
#include <eeduro/hal/Output.hpp>
#include <eeduro/eeduro.hpp>
#include <eeros/control/Block1i1o.hpp>
#include <eeros/types.hpp>

namespace eeduro {
	
	class Board : public eeros::control::Block1i1o<AxisVector> {
	
	public:
		Board();
		virtual ~Board();

		virtual bool open(const char *dev);
		virtual void close();

		virtual void run();

		virtual void setEnable(bool value);
		virtual void setEnable(int axis, bool value);
		virtual void setReset(bool value);
		virtual void setReset(int axisPair, bool value);

		virtual void limit(double voltage);
		virtual void resetEmergency();

		bool transmission_ok;
		bool power_out[2];
		bool button[2];
		bool reset[2];

		Latch button_latch[2] = { button[0], button[1] };

		struct{
			bool enable;
			bool fault;
			double position;
			bool current_limit[2];
		} axis[nofAxis];
	
	private:
		int fd;

		double voltage_limit = 12;
		const double max_voltage = 12;
		const int encoder_ticks = 512 * 4;
		const double gear = 387283.0 / 5103.0;
		const double k = 2 * 3.1415926535897932384626433832795 / ( static_cast<double>(encoder_ticks) * gear );

		const uint8_t  mode;
		const uint8_t  lsb_first;
		const uint8_t  bits_per_word;
		const uint32_t speed_Hz;

		struct {
			bool direction;
			int16_t position;
			uint16_t duty;
			double voltage;
		} _axis[nofAxis];

		Latch emergency_latch = { button[0] };
		Input<bool> emergency = { "emergency", emergency_latch.state };

		Input<bool> fault[nofAxis] = {
			{ "fault0", axis[0].fault },
			{ "fault1", axis[1].fault },
			{ "fault2", axis[2].fault },
			{ "fault3", axis[3].fault }
		};

		Input<double> position[nofAxis] = {
			{ "q0", axis[0].position },
			{ "q1", axis[1].position },
			{ "q2", axis[2].position },
			{ "q3", axis[3].position }
		};

		Output<bool> enable[nofAxis] = {
			{ "enable0", axis[0].enable },
			{ "enable1", axis[1].enable },
			{ "enable2", axis[2].enable },
			{ "enable3", axis[3].enable }
		};

		Output<double> voltage[nofAxis] = {
			{ "voltage0", _axis[0].voltage },
			{ "voltage1", _axis[1].voltage },
			{ "voltage2", _axis[2].voltage },
			{ "voltage3", _axis[3].voltage }
		};
	};
}

#endif /* __CH_NTB_EEDURO_BOARD_HPP_ */

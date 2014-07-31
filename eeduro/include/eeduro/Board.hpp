#ifndef __CH_NTB_EEDURO_BOARD_HPP_
#define __CH_NTB_EEDURO_BOARD_HPP_

#include <eeduro/Latch.hpp>
#include <eeduro/Input.hpp>
#include <eeduro/Output.hpp>
#include <eeros/control/Block1i1o.hpp>
#include <eeros/math/Matrix.hpp>

#define NOF_AXIS (4)

namespace eeduro {
	
	class Board : public eeros::control::Block1i1o<eeros::math::Matrix<NOF_AXIS, 1, double>> {
	
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
		virtual void resetPositions();
		
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
		} axis[NOF_AXIS];
	
	private:
		int fd;

		double voltage_limit = 12;
		const double max_voltage = 12;
		const int encoder_ticks = 512 * 4;
//		const double gear = 387283.0 / 5103.0;
		const double k = 2 * 3.1415926535897932384626433832795 / ( static_cast<double>(encoder_ticks) );

		const uint8_t  mode;
		const uint8_t  lsb_first;
		const uint8_t  bits_per_word;
		const uint32_t speed_Hz;

		struct {
			bool direction;
			int16_t position;
			uint16_t duty;
			double voltage;
		} _axis[NOF_AXIS];
		
		bool clearPosition[NOF_AXIS];

		Latch emergency_latch = { button[0] };
		eeduro::hal::Input<bool> emergency = { "emergency", emergency_latch.state };

		eeduro::hal::Input<bool> fault[NOF_AXIS] = {
			{ "fault0", axis[0].fault },
			{ "fault1", axis[1].fault },
			{ "fault2", axis[2].fault },
			{ "fault3", axis[3].fault }
		};

		eeduro::hal::Input<double> position[NOF_AXIS] = {
			{ "q0", axis[0].position },
			{ "q1", axis[1].position },
			{ "q2", axis[2].position },
			{ "q3", axis[3].position }
		};

		eeduro::hal::Output<bool> enable[NOF_AXIS] = {
			{ "enable0", axis[0].enable },
			{ "enable1", axis[1].enable },
			{ "enable2", axis[2].enable },
			{ "enable3", axis[3].enable }
		};

		eeduro::hal::Output<double> voltage[NOF_AXIS] = {
			{ "voltage0", _axis[0].voltage },
			{ "voltage1", _axis[1].voltage },
			{ "voltage2", _axis[2].voltage },
			{ "voltage3", _axis[3].voltage }
		};
	};
}

#endif /* __CH_NTB_EEDURO_BOARD_HPP_ */

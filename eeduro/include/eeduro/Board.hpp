#ifndef __CH_NTB_EEDURO_BOARD_HPP_
#define __CH_NTB_EEDURO_BOARD_HPP_

#include <eeduro/Latch.hpp>
#include <eeduro/Input.hpp>
#include <eeduro/Output.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/math/Matrix.hpp>

#define NOF_AXIS (4)

namespace eeduro {
	
	class Board : public eeros::control::Block {
	
	public:
		Board();
		virtual ~Board();

		virtual bool open(const char *dev);
		virtual void close();

		virtual void run();

		virtual eeros::control::Input<eeros::math::Matrix<NOF_AXIS, 1, double>>& getIn();
		virtual eeros::control::Output<eeros::math::Matrix<NOF_AXIS, 1, double>>& getPosOut();
		virtual eeros::control::Output<eeros::math::Matrix<NOF_AXIS, 1, double>>& getSpeedOut();
		
		virtual void setEnable(bool value);
		virtual void setEnable(int axis, bool value);
		virtual void setReset(bool value);
		virtual void setReset(int axisPair, bool value);

		virtual void limit(double voltage);
		virtual void resetPositions(double q0 = 0, double q1 = 0, double q2 = 0, double q3 = 0);
		
		bool transmission_ok;
		bool power_out[2];
		bool button[3];
		bool reset[2];
		bool led[4];

		Latch button_latch[3] = { button[0], button[1], button[2] };

		struct{
			bool enable;
			bool fault;
			double position;
			double speed;
			bool current_limit[2];
		} axis[NOF_AXIS];
	
	private:
		eeros::control::Input<eeros::math::Matrix<NOF_AXIS, 1, double>> voltageIn;
		eeros::control::Output<eeros::math::Matrix<NOF_AXIS, 1, double>> posOut;
		eeros::control::Output<eeros::math::Matrix<NOF_AXIS, 1, double>> speedOut;
		
		int fd;

		double voltage_limit = 12;
		const double max_voltage = 12;
		const int encoder_ticks[4] = { 4096 * 4, 4096 * 4, 4096 * 4, 256 * 4 };
		static constexpr double pi = 3.1415926535897932384626433832795;
		const double k[4] = {	2 * pi / ( static_cast<double>(encoder_ticks[0]) ),
								2 * pi / ( static_cast<double>(encoder_ticks[1]) ),
								2 * pi / ( static_cast<double>(encoder_ticks[2]) ),
								2 * pi / ( static_cast<double>(encoder_ticks[3]) ) };

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
		double initPosition[NOF_AXIS];
		
		eeros::control::Signal<eeros::math::Matrix<NOF_AXIS, 1, double>> prevPos;
		
		eeduro::hal::Input<bool> emergency = { "emergency", button[1] };
		
		eeduro::hal::Input<bool> approval = { "approval", button[2] };

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

		eeduro::hal::Output<bool> led_output[NOF_AXIS] = {
			{ "led0", led[0] },
			{ "led1", led[1] },
			{ "led2", led[2] },
			{ "led3", led[3] }
		};
	};
}

#endif /* __CH_NTB_EEDURO_BOARD_HPP_ */

#include <eeduro/Board.hpp>

using namespace eeduro;

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <eeros/hal/HAL.hpp>

Board::Board() :
		fd(-1),
		mode(SPI_MODE_2),
		lsb_first(0),
		bits_per_word(32),
		speed_Hz(500000),
		transmission_ok(false)
{
	for (int i = 0; i < NOF_AXIS; i++) {
		power_out[0] = false;
		power_out[1] = false;
		button[0] = false;
		button[1] = false;
		reset[0] = true;
		reset[1] = true;

		axis[i].position = 0;
		axis[i].enable = false;
		axis[i].fault = false;
		axis[i].current_limit[0] = false;
		axis[i].current_limit[1] = false;

		_axis[i].direction = true;
		_axis[i].position = 0;
		_axis[i].duty = 0;
		_axis[i].voltage = 0;
	}

	resetEmergency();

	eeros::hal::HAL& hal = eeros::hal::HAL::instance();
	hal.addPeripheralInput(&emergency);
	for (int i = 0; i < NOF_AXIS; i++) {
		hal.addPeripheralInput(&fault[i]);
		hal.addPeripheralInput(&position[i]);
		hal.addPeripheralOutput(&enable[i]);
		hal.addPeripheralOutput(&voltage[i]);
	}
}

Board::~Board() { ::close(fd); }

void Board::close() { ::close(fd); }

bool Board::open(const char *dev) {
	fd = ::open(dev, O_RDWR);

	if (fd >= 0) {
		int r;
		int result = -2; // internal error

		r = ioctl(fd, SPI_IOC_WR_MODE, &mode);
		if (r != -1) {
			r = ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb_first);
			if (r != -1) {
				r = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
				if (r != -1) {
					r = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_Hz);
					if (r != -1) {
						return true;
					}
					else {
						result = -6; // cannot set speed
					}
				}
				else {
					result = -5; // cannot set bits per word
				}
			}
			else {
				result = -4; // cannot set SPI LSB_FIRST property
			}
		}
		else {
			result = -3; // cannot set SPI mode
		}

		::close(fd);
		fd = (-1);
		return false;
	}
	return false; // cannot open device
}

void Board::run() {

	static uint64_t timestamp = 0;
	timestamp += 1000000; // 1 ms // TODO -> use system time

	int r;

	uint32_t read_data;
	uint32_t write_data;

	spi_ioc_transfer tr;
	tr.len = 4;
	tr.delay_usecs = 0;
	tr.speed_hz = speed_Hz;
	tr.bits_per_word = bits_per_word;

	bool invert[NOF_AXIS] = { false, false, false, true };
	bool axis_ok[NOF_AXIS];
	for (int i = 0; i < NOF_AXIS; i++) axis_ok[i] = false;

	for (int i = 0; i < NOF_AXIS; i++) {
		read_data = 0;

		_axis[i].voltage = in.getSignal().getValue()(i);

		double v = 0;

		if (_axis[i].voltage >= 0) {
			v = _axis[i].voltage;
			_axis[i].direction = false;
		}
		else {
			v = -_axis[i].voltage;
			_axis[i].direction = true;
		}

		if (v > voltage_limit)
			v = voltage_limit;

		if (v > max_voltage)
			v = max_voltage;

		if (v < 0)
			v = 0;

		_axis[i].duty = ((uint16_t)((v/max_voltage)*0xffff)) & 0xffff;

		write_data = (
						((i & 0xf) << 28) |
						(axis[i].enable << 27) |
						((invert[i] ^ _axis[i].direction) << 26) |
						(reset[1] << 21) |
						(reset[0] << 20) |
						(axis[i].current_limit[1] << 19) |
						(axis[i].current_limit[0] << 18) |
						((power_out[1] ^ true) << 17) |
						((power_out[0] ^ true) << 16) |
						(_axis[i].duty & 0xffff)
					);

		tr.rx_buf = (unsigned long)&read_data;
		tr.tx_buf = (unsigned long)&write_data;

		r = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

		int a = ((read_data >> 28) & 0xf);
		if (a >= 0 && a < NOF_AXIS) {
			axis_ok[a] = true;
			axis[a].fault = ((read_data >> 25) & 0x1);

			button[0] = ((read_data >> 20) & 0x1);
			emergency_latch.run();
			button_latch[0].run();

			button[1] = ((read_data >> 21) & 0x1);
			button_latch[1].run();

			int16_t old = _axis[a].position;
			_axis[a].position = (read_data & 0xffff);
			int16_t delta = (_axis[a].position - old);
			axis[a].position += k * delta;

			eeros::math::Matrix<4,1> o = out.getSignal().getValue();
			o(a) = axis[a].position;
			out.getSignal().setValue(o);
			out.getSignal().setTimestamp(timestamp);
		}
	}

	bool all_ok = true;

	for (int i = 0; i < NOF_AXIS; i++)
		if (!axis_ok[i]) all_ok = false;

	transmission_ok = all_ok;
}

void Board::setEnable(bool value) {
	for (int i = 0; i < 4; i++)
		axis[i].enable = value;
}

void Board::setEnable(int axis, bool value) {
	this->axis[axis].enable = value;
}

void Board::setReset(bool value) {
	for (int i = 0; i < 2; i++)
		this->reset[i] = value;
}

void Board::setReset(int axisPair, bool value) {
	this->reset[axisPair] = value;
}

void Board::limit(double voltage) {
	if (voltage >= 0)
		voltage_limit = voltage;
}

void Board::resetEmergency() {
	emergency_latch.state = false;
}

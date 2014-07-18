#ifndef EEDURO_HPP_
#define EEDURO_HPP_

#include <eeros/math/Matrix.hpp>

namespace eeduro {

	// Math constants
	static const double pi = 3.14159265359;
	
	// General constants
	static const unsigned int nofAxis = 4;
	
	// Types
	typedef eeros::math::Matrix<nofAxis, 1> AxisVector;
	
	// Mechanical and electrical characteristics
	static const double kM0 = 11.4e-3; // Nm/A
	static const double kM1 = kM0;
	static const double kM2 = kM0;
	static const double kM3 = 8.98e-3; // Nm/A
	
	// Controller parameters
	static const double dt = 0.001; // [s]
	static const double D = 0.7;
	static const double w0 = 2 * pi * 1 / (2 * D * dt * 2 * pi);
	static const double kp = w0 / (2 * D);
	static const double kd = 2 * D * w0;
	
}
#endif // EEDURO_HPP_

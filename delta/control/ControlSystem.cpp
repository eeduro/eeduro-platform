#include "ControlSystem.hpp"

#include <eeros/core/EEROSException.hpp>
#include <unistd.h>

using namespace eeduro::delta;
using namespace eeros;
using namespace eeros::control;

ControlSystem::ControlSystem() :
	posSetPoint(0),
	posController(kp),
	speedSetPoint(0),
	speedController(kd),
	inertia(1), // TODO
//	jacobi, // TODO
	invGear(1.0/i),
	gear(i),
//	motorModel, // TODO
//	directKin, // TODO
	timedomain("Main time domain", dt, true) {

	i << i1524, i1524, i1524, i0816;
	posSum.negateInput(1);
	speedSum.negateInput(1);
	
	board.getIn().connect(motorModel.getOut());
	posSum.getIn(0).connect(posSetPoint.getOut());
	posSum.getIn(1).connect(directKin.getOut());
	posController.getIn().connect(posSum.getOut());
	posDiff.getIn().connect(directKin.getOut());
	speedSum.getIn(0).connect(posController.getOut());
	speedSum.getIn(1).connect(posDiff.getOut());
	speedSum.getIn(2).connect(speedSetPoint.getOut());
	speedLimitation.getIn().connect(speedSum.getOut());
	speedController.getIn().connect(speedLimitation.getOut());
	inertia.getIn().connect(speedController.getOut());
	forceLimitation.getIn().connect(inertia.getOut());
	jacobi.getIn().connect(forceLimitation.getOut());
	torqueLimitation.getIn().connect(jacobi.getOut());
	invGear.getIn().connect(torqueLimitation.getOut());
	gear.getIn().connect(board.getOut());
	motorModel.getTorqueIn().connect(invGear.getOut());
	motorModel.getSpeedIn().connect(angleDiff.getOut());
	angleDiff.getIn().connect(board.getOut());
	directKin.getIn().connect(gear.getOut());
	
	timedomain.addBlock(&board);
	timedomain.addBlock(&angleDiff);
	timedomain.addBlock(&gear);
	timedomain.addBlock(&directKin);
	timedomain.addBlock(&posDiff);
	timedomain.addBlock(&posSetPoint);
	timedomain.addBlock(&speedSetPoint);
	timedomain.addBlock(&posSum);
	timedomain.addBlock(&posController);
	timedomain.addBlock(&speedSum);
	timedomain.addBlock(&speedLimitation);
	timedomain.addBlock(&speedController);
	timedomain.addBlock(&inertia);
	timedomain.addBlock(&forceLimitation);
	timedomain.addBlock(&jacobi);
	timedomain.addBlock(&torqueLimitation);
	timedomain.addBlock(&invGear);
	timedomain.addBlock(&motorModel);
}

void ControlSystem::start() {
	timedomain.start();
}

void ControlSystem::stop() {
	timedomain.stop();
	timedomain.join();
}

void ControlSystem::enableAxis() {
	board.setEnable(true);
	board.setReset(false);
}

void ControlSystem::disableAxis() {
	board.setEnable(false);
	board.setReset(true);
}

void ControlSystem::resetEncoders() {
	board.setReset(1, true);
	usleep(2000);
	board.setReset(1, false);
}

void ControlSystem::goToPos(double x, double y, double z, double phi) {
	AxisVector p;
	p << x, y, z, phi;
	posSetPoint.setValue(p);
}

void ControlSystem::initBoard() {
	if(!board.open("/dev/spidev1.0"))
		throw EEROSException("failed to open SPI device");
}

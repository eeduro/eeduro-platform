#include "ControlSystem.hpp"

#include <eeros/core/EEROSException.hpp>
#include <unistd.h>

using namespace eeduro::delta;
using namespace eeros;
using namespace eeros::control;

ControlSystem::ControlSystem() :
	i(i1524, i1524, i1524, i0816),
	kM(kM1524, kM1524, kM1524, kM0816),
	RA(RA1524, RA1524, RA1524, RA0816),
	
	homed(false),
	posController(kp),
	speedController(kd),
	inertia(jred),
	jacobi(jacobian),
	motorModel(kM, RA),
	voltageSwitch(1),
	directKin(kinematic),
	timedomain("Main time domain", dt, true) {

	torqueLimitation.enable();
	torqueGear.setGain(1.0 / i);
	angleGear.setGain(1.0 / i);
	
	posSum.negateInput(1);
	speedSum.negateInput(1);
	
	board.getIn().connect(voltageSwitch.getOut());
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
	jacobi.getForceInput().connect(forceLimitation.getOut());
	jacobi.getJointPosInput().connect(angleGear.getOut());
	jacobi.getTcpPosInput().connect(directKin.getOut());
	torqueLimitation.getIn().connect(jacobi.getOut());
	torqueGear.getIn().connect(torqueLimitation.getOut());
	angleGear.getIn().connect(board.getPosOut());
	motorModel.getTorqueIn().connect(torqueGear.getOut());
	motorModel.getSpeedIn().connect(board.getSpeedOut());
	voltageSwitch.getIn(0).connect(motorModel.getOut());
	voltageSwitch.getIn(1).connect(voltageSetPoint.getOut());
	directKin.getIn().connect(angleGear.getOut());
	
	timedomain.addBlock(&board);
	timedomain.addBlock(&angleGear);
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
	timedomain.addBlock(&torqueGear);
	timedomain.addBlock(&motorModel);
	timedomain.addBlock(&voltageSetPoint);
	timedomain.addBlock(&voltageSwitch);
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

// void ControlSystem::initAxis() {
// 	if(initialized) return;
// 	
// 	voltageSetPoint.setValue({2.5, 2.5, 2.5, 1.5});
// 	voltageSwitch.switchToInput(1);
// 	
// 	do {
// //		std::this_thread::sleep_for(std::chrono::microseconds(500)); // TODO doesn't compiles with linaro toolchain...
// 		sleep(1);
// 	} while(!allAxisStopped());
// 	
// 	board.resetPositions();
// 	voltageSetPoint.setValue({0, 0, 0, 0});
// 	voltageSwitch.switchToInput(0);
// 	initialized = true;
// }

void ControlSystem::setVoltageForInitializing(AxisVector u) {
	voltageSetPoint.setValue(u);
}

bool ControlSystem::switchToPosControl() {
	if(homed || !allAxisStopped()) return false;
	board.resetPositions();
	setVoltageForInitializing({0, 0, 0, 0});
	voltageSwitch.switchToInput(0);
	homed = true;
	return true;
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

AxisVector ControlSystem::getTcpPos() {
	return directKin.getOut().getSignal().getValue();
}

AxisVector ControlSystem::getAxisPos() {
	return angleGear.getOut().getSignal().getValue();
}

bool ControlSystem::allAxisStopped(double maxSpeed) {
	for(int i = 0; i < nofAxis; i++) {
		if(board.getSpeedOut().getSignal().getValue()[i] > maxSpeed) return false;
	}
	return true;
}

bool ControlSystem::axisHomed() {
	return homed;
}

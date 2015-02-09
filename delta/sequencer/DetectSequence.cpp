#include "DetectSequence.hpp"
#include "../control/constants.hpp"
#include <unistd.h>

using namespace eeduro::delta;
using namespace eeros::sequencer;


DetectSequence::DetectSequence(Sequencer* sequencer, ControlSystem* controlSys) :
	Sequence<int, int>("detect", sequencer),
	controlSys(controlSys)
{
	calibration.loadDefaults();
	if (!calibration.load()) {
		log.warn() << "could not load calibration";
	}
}

int DetectSequence::run(int position) {
	double down = calibration.position[position].level12 + 0.002;
	double touch = calibration.position[position].level30 - 0.0002;
	
	eeros::math::Vector<4> torqueLimit{ q012gearTorqueLimit, q012gearTorqueLimit, q012gearTorqueLimit, q3gearTorqueLimit };
	eeros::math::Vector<4> torqueLimitDown = torqueLimit * 0.1;
	eeros::math::Vector<4> zero = torqueLimit * 0.01;
	
	auto p = controlSys->pathPlanner.getLastPoint();
	double last_z = p[2];
	p[2] = down;
	controlSys->pathPlanner.gotoPoint(p);
	waitUntilPointReached();
	
	controlSys->torqueLimitation.setLimit(-torqueLimitDown, torqueLimitDown);
	
	p[2] = touch;
	controlSys->pathPlanner.gotoPoint(p);
	waitUntilPointReached();
	
	controlSys->torqueLimitation.setLimit(-zero, zero);
	usleep(500000);
	double z = controlSys->directKin.getOut().getSignal().getValue()[2];
	controlSys->torqueLimitation.setLimit(-torqueLimitDown, torqueLimitDown);
	
	p[2] = down;
	controlSys->pathPlanner.gotoPoint(p);
	waitUntilPointReached();
	
	controlSys->torqueLimitation.setLimit(-torqueLimit, torqueLimit);
	
	p[2] = last_z;
	controlSys->pathPlanner.gotoPoint(p);
	waitUntilPointReached();
	
	int block = calibration.getBlock(position, z);
	log.trace() << "[DETECT] pos " << position << ": z = " << z << " -> block = " << block;
	
	return block;
}

void DetectSequence::waitUntilPointReached() {
	while (!controlSys->pathPlanner.posReached()) {
		usleep(100000);
		yield();
	}
}

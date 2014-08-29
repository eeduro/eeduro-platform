#include "MoveBlockSequence.hpp"
using namespace eeduro::delta;
using namespace eeros::sequencer;
using namespace eeros::safety;

enum {
	idle,
	move_from_to,
	move_to
};

MoveBlockSequence::MoveBlockSequence(Sequencer* sequencer, ControlSystem* controlSys, SafetySystem* safetySys) : Sequence<void, int, int>("main", sequencer), controlSys(controlSys), safetySys(safetySys), state(idle) {
	// nothing to do
}

bool MoveBlockSequence::checkPreCondition() {
	return (state != idle);
}

void MoveBlockSequence::run(int from, int to) {
	if (state == idle) {
		this->from = from;
		this->to = to;
		state = move_from_to;
	}
	else {
		log.warn() << "[" << name << "]: another operation is currently executing";
	}
}

void MoveBlockSequence::run(int to) {
	if (state == idle) {
		this->from = (-1);
		this->to = to;
		state = move_to;
	}
	else {
		log.warn() << "[" << name << "]: another operation is currently executing";
	}
}

void MoveBlockSequence::init() {
	
	std::bind(&MoveBlockSequence::init, *this);
	
// 	if (state == move_from_to) {
// 		addStep([&] () { up(); });
// 		addStep([&] () { move(from); });
// 		addStep([&] () { down(); });
// 		addStep([&] () { grab(); });
// 	}
// 	
// 	addStep([&] () { up(); });
// 	addStep([&] () { move(to); });
// 	addStep([&] () { down(); });
// 	addStep([&] () { release(); });
// 	addStep([&] () { up(); });
}

void MoveBlockSequence::exit() {
	state = idle;
}

void MoveBlockSequence::up() {
	log.trace() << "move to transportation height";
}

void MoveBlockSequence::down() {
	log.trace() << "move down";
}

void MoveBlockSequence::grab() {
	log.trace() << "grab block";
}

void MoveBlockSequence::release() {
	log.trace() << "release block";
}

void MoveBlockSequence::move(int position) {
	log.trace() << "move to position " << position;
}



#include <eeduro/control/ControlSystem.hpp>

using namespace eeduro;
using namespace eeros::control;

ControlSystem::ControlSystem(double ts) :  setpointPos(0), setpointSpeed(0), posController(kp),switchSpeed(0),
	speedController(kd), inertia(1), invMotConst(1 / kM), timedomain("Main time domain", ts, true) {

	kM << kM0,   0,   0,   0,
	        0, kM1,   0,   0,
			0,   0, kM2,   0,
			0,   0,   0, kM3;
	
	sumPos.negateInput(1);
	sumSpeed.negateInput(1);
	
	diffSetPointPos.getIn().connect(setpointPos.getOut());
	sumPos.getIn(0).connect(setpointPos.getOut());
	sumPos.getIn(1).connect(board.getOut());
	posController.getIn().connect(sumPos.getOut());
	diffEncPos.getIn().connect(board.getOut());
	sumSpeed.getIn(0).connect(posController.getOut());
	sumSpeed.getIn(1).connect(diffEncPos.getOut());
	sumSpeed.getIn(2).connect(diffSetPointPos.getOut());
	switchSpeed.getIn(0).connect(setpointSpeed.getOut());
	switchSpeed.getIn(1).connect(sumSpeed.getOut());
	speedController.getIn().connect(switchSpeed.getOut());
	inertia.getIn().connect(speedController.getOut());
	invMotConst.getIn().connect(inertia.getOut());
	board.getIn().connect(invMotConst.getOut());
	
	timedomain.addBlock(&setpointPos);
	timedomain.addBlock(&setpointSpeed);
	timedomain.addBlock(&sumPos);
	timedomain.addBlock(&posController);
	timedomain.addBlock(&sumSpeed);
	timedomain.addBlock(&speedController);
	timedomain.addBlock(&inertia);
	timedomain.addBlock(&invMotConst);
	timedomain.addBlock(&board);
}

void ControlSystem::start() {
	timedomain.start();
}

void ControlSystem::stop() {
	timedomain.stop();
	timedomain.join();
}

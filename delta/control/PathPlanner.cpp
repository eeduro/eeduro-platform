#include "PathPlanner.hpp"
#include <eeros/core/System.hpp>
#include <iostream>

using namespace eeduro::delta;
using namespace eeros;

PathPlanner::PathPlanner(AxisVector velMax, AxisVector accMax, double dt) : trajectoryGen(velMax, accMax, -accMax, dt) { }

Output<AxisVector>& PathPlanner::getPosOut() {
	return posOut;
}

Output<AxisVector>& PathPlanner::getVelOut() {
	return velOut;
}

Output<AxisVector>& PathPlanner::getAccOut() {
	return accOut;
}

void PathPlanner::gotoPoint(AxisVector p) {
// 	AxisVector z; z.zero();
// 	std::array<AxisVector, 3> end;
// 	for(auto& e : end) e = z;
// 	std::lock_guard<std::mutex> lck(mtx);
// 	end[0] = p;
	TrajectoryGenerator<AxisVector, 3>* t = static_cast<TrajectoryGenerator<AxisVector, 3>*>(&trajectoryGen);
	t->push(p);
	// 	trajectoryGen.push(p);
}

bool PathPlanner::posReached() {
	return trajectoryGen.finished();
}

void PathPlanner::setInitPos(AxisVector initPos) {
	AxisVector z; z.zero();
	std::array<AxisVector, 3> r;
	r[0] = initPos;
	r[1] = z;
	r[2] = z;
	trajectoryGen.reset(r);
}

void PathPlanner::run() {
	std::array<AxisVector, 3> x;
	double t = System::getTime();
	x = trajectoryGen.get(dt); // TODO use exact dt
	
// 	static int j = 0;
// 	if (j++ > 300) {
// 		std::cout << x[0] << std::endl;
// 		j = 0;
// 	}
	
	posOut.getSignal().setValue(x[0]);
	posOut.getSignal().setTimestamp(t);
	velOut.getSignal().setValue(x[1]);
	velOut.getSignal().setTimestamp(t);
	accOut.getSignal().setValue(x[2]);
	accOut.getSignal().setTimestamp(t);
}

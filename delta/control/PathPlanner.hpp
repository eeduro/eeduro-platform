#ifndef CH_NTB_EEDURO_DELTA_PATHPLANNER_HPP_
#define CH_NTB_EEDURO_DELTA_PATHPLANNER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/ConstantAccTrajectoryGenerator.hpp>
#include <mutex>
#include "types.hpp"

using namespace eeros;
using namespace eeros::control;

namespace eeduro {
	namespace delta {
		
		class PathPlanner: public Block {
			
		public:
			PathPlanner(AxisVector velMax, AxisVector accMax, double dt);
			
			virtual Output<AxisVector>& getPosOut();
			virtual Output<AxisVector>& getVelOut();
			virtual Output<AxisVector>& getAccOut();
			
			virtual void gotoPoint(AxisVector p);
			virtual bool posReached();
			virtual void setInitPos(AxisVector initPos);
			virtual AxisVector getLastPoint();
			
			virtual void run();
			
		protected:
			Output<AxisVector> posOut;
			Output<AxisVector> velOut;
			Output<AxisVector> accOut;
			std::mutex mtx;
			double tOld;
			
			ConstantAccTrajectoryGenerator<AxisVector> trajectoryGen;
			AxisVector lastPoint;
		};
	}
}

#endif /* CH_NTB_EEDURO_DELTA_PATHPLANNER_HPP_ */
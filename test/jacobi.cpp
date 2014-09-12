#include <iostream>
#include <eeros/math/Matrix.hpp>
#include "../delta/control/Kinematic.hpp"
#include "../delta/control/Jacobian.hpp"
#include "../delta/control/NumericalJacobian.hpp"

using namespace std;
using namespace eeduro::delta;
using namespace eeros::math;

double diff(const Vector3 &a, const Vector3 &b)
{	
	Vector3 diff = (a - b);
	double d = 0;
	for (int i = 0; i < 3; i++) 
	{
		if (diff[i] < 0)
			d += -diff[i];
		else
			d += diff[i];
	}
	return d;
}

double norm(const Vector3 &a)
{
	return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

int main(int argc, char *argv[])
{
	cout << "jacobi test started" << endl;
	
//  	double qmin = -1.74;
	double qmin = -0.4;
	double qmax = 0.4;
	
	int n = 200;
	double dq = (qmax - qmin) / (double)n;
	cout << "dq: " << dq << endl;
	
	Kinematic k;
	Jacobian j(k.get_offset());
	NumericalJacobian<3,3> num(k, 0.000001);
	
	double max_angle = 0;
	double max_amplitude = 0;
	
	try
	{
		Vector3 qlast(qmin, qmin, qmin);
		Vector3 xlast;
		k.forward(qlast, xlast);
		for (int i0 = 0; i0 < n; i0++)
		{
			for (int i1 = 0; i1 < n; i1++)
			{
				for (int i2 = 0; i2 < n; i2++)
				{
					Vector3 q( i0 * dq + qmin, i1 * dq + qmin, i2 * dq + qmin );
					Vector3 x;
					
					if (!k.forward(q, x))
					{
						cout << "q: " << q << endl;
						throw 0;
					}
					
					if (i2 == 0)
					{
						qlast = q;
						xlast = x;
						continue;
					}
					
					Vector3 qback;
					if (!k.inverse(x, qback))
					{
						cout << "FAIL:" << endl;
						cout << "q: " << q << endl;
						cout << "tcp: " << x << endl;
						throw 0;
					}
					
					double d = diff(q, qback);
					
					if (d > 1e-12)
					{
						cout << "FAIL:" << endl;
						cout << "q: " << q << endl;
						cout << "tcp: " << x << endl;
						cout << "d: " << d << endl;
						throw 1;
					}
					
					if (!j.calculate(q, x))
					{
						cout << "FAIL:" << endl;
						cout << "q: " << q << endl;
						cout << "tcp: " << x << endl;
						cout << j.getJacobian() << endl;
						throw 2;
					}
					
					Matrix<3,3> J = j.getJacobian();
					
					if (!num.calculate(q))
					{
						cout << "FAIL:" << endl;
						cout << "q: " << q << endl;
						cout << "tcp: " << x << endl;
						throw 3;
					}
					
					Matrix<3,3> Jnum = num.getJacobian();
					Matrix<3,3> dJ = (Jnum - J);
					
					Matrix<3,1> one;
					one = 1;
					
					Matrix<3,1> one1 = (!Jnum) * (J * one);
					Matrix<3,1> one2 = (!J) * (Jnum * one);
					
					d = diff(one1, one2);
					
					if (d > 0.001) {
						cout << "FAIL:" << endl;
						cout << "q: " << q << endl;
						cout << "d: " << d << endl;
						cout << "J: " << J << endl;
						cout << "Jnum: " << Jnum << endl;
						cout << "dJ: " << dJ << endl;
						cout << "one1: " << one1 << endl;
						cout << "one2: " << one2 << endl;
						throw 4;
					}
					
					double d1 = diff(J.getSubMatrix<3,1>(0,0), Jnum.getSubMatrix<3,1>(0,0));
					double d2 = diff(J.getSubMatrix<3,1>(0,1), Jnum.getSubMatrix<3,1>(0,1));
					double d3 = diff(J.getSubMatrix<3,1>(0,2), Jnum.getSubMatrix<3,1>(0,2));
					
					d = d1;
					d += d2;
					d += d3;
					
					if (d > 0.001) {
						cout << "FAIL:" << endl;
						cout << "q: " << q << endl;
						cout << "d: " << d << endl;
						cout << "J: " << J << endl;
						cout << "Jnum: " << Jnum << endl;
						cout << "dJ: " << dJ << endl;
						throw 5;
					}
					
// 					J = Jnum;
					
					Vector3 qdot((q - qlast) / 0.001);
					Vector3 xdot((x - xlast) / 0.001);
					
					Vector3 xdot_calc = J * qdot;
					
					d = diff(xdot, xdot_calc);

					double xnorm = norm(xdot);
					double calcnorm = norm(xdot_calc);
					double nd = (xnorm - calcnorm);
					double s = acos((xdot.transpose() * xdot_calc) / xnorm / calcnorm) * 180.0 / 3.14159;
					if (s < 0) s = -s;
					
					if (s > max_angle) max_angle = s;
					
					d = d / xnorm;
					if (d > max_amplitude) max_amplitude = d;
					
// 					if (s > 1 || d > 0.01)
// 					{
// 						cout << "FAIL:\t" << endl;
// 						cout << "q:\t" << q << endl;
// 						cout << "tcp:\t" << x << endl;
// 						cout << "qdot:\t" << qdot << endl;
// 						cout << "xdot:\t" << xdot << endl;
// 						cout << "xdot_calc:\t" << xdot_calc << endl;
// 						cout << "xnorm:\t" << xnorm << endl;
// 						cout << "d:\t" << d << endl;
// 						cout << "nd:\t" << nd << endl;
// 						cout << "s:\t" << s << endl;
// 						throw 6;
// 					}
					
					qlast = q;
					xlast = x;
				}
				
				int percentage = (100 * (i0 * n + i1)) / (n*n);
				if ((percentage % 10) == 0) {
					static int last = -1;
					if (last != percentage) {
						last = percentage;
						cout << percentage << " %" << endl;
					}
				}
			}
		}
		
		cout << "SUCCESS:" << endl;
		cout << "   max. angle deviation:     " << max_angle << " rad" << endl;
		cout << "   max. amplitude deviation: " << (max_amplitude * 100) << " %" << endl;
		cout << "   points evaluated:         " << (n*n*n) << endl;
	}
	catch(int &ex)
	{
		cout << "EXCEPTION: " << ex << endl;
	}
	
	cout << "jacobi test finished" << endl;
	return 0;
}
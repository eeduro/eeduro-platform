#include <iostream>
using namespace std;

#include "Kinematic.hpp"
#include "Jacobian.hpp"
using namespace eeduro::delta;

#include <eeros/math/Matrix.hpp>
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
	
// 	double qmin = -1.74;
	double qmin = -0.4;
	double qmax = 0.4;
	
	int n = 100;
	double dq = (qmax - qmin) / (double)n;
	
	
	Kinematic k;
	Jacobian j(k.get_offset());
	
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
					
					Vector3 qdot((q - qlast) / 0.001);
					Vector3 xdot((x - xlast) / 0.001);
					
					Vector3 xdot_calc = J * qdot;
					
					d = diff(xdot, xdot_calc);

					double xnorm = norm(xdot);
					double calcnorm = norm(xdot_calc);
					double nd = (xnorm - calcnorm);
					double s = acos((xdot.transpose() * xdot_calc) / xnorm / calcnorm) * 180.0 / 3.14159;
					if (s < 0) s = -s;
					
// 					if (d > xnorm * 0.01)
					if (s > 1)
					{
						cout << "FAIL:\t" << endl;
						cout << "q:\t" << q << endl;
						cout << "tcp:\t" << x << endl;
						cout << "qdot:\t" << qdot << endl;
						cout << "xdot:\t" << xdot << endl;
						cout << "xdot_calc:\t" << xdot_calc << endl;
						cout << "xnorm:\t" << xnorm << endl;
						cout << "d:\t" << d << endl;
						cout << "nd:\t" << nd << endl;
						cout << "s:\t" << s << endl;
						throw 3;
					}
					
					qlast = q;
					xlast = x;
				}
			}
		}
	}
	catch(int &ex)
	{
		cout << "EXCEPTION: " << ex << endl;
	}
	
	cout << "jacobi test finished" << endl;
	return 0;
}
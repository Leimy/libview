#include <u.h>
#include <libc.h>
#include "view.h"

/*
 * Vector helpers, ported unchanged from claudegraph's
 * vadd/vsub/vmul/vdot/rndvec (and equivalent to radioglobe's
 * open-coded dot products).
 */

Vec3
v3(double x, double y, double z)
{
	Vec3 v;

	v.x = x;
	v.y = y;
	v.z = z;
	return v;
}

Vec3
v3add(Vec3 a, Vec3 b)
{
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return a;
}

Vec3
v3sub(Vec3 a, Vec3 b)
{
	a.x -= b.x;
	a.y -= b.y;
	a.z -= b.z;
	return a;
}

Vec3
v3scale(Vec3 a, double s)
{
	a.x *= s;
	a.y *= s;
	a.z *= s;
	return a;
}

double
v3dot(Vec3 a, Vec3 b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

double
v3len(Vec3 a)
{
	return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

/*
 * Uniform-direction vector of length r, for jitter and initial
 * placement.  Ported from claudegraph's rndvec(); uses frand(),
 * so seed with srand() in the application if determinism matters.
 */
Vec3
v3rand(double r)
{
	Vec3 v;
	double m;

	do{
		v.x = frand() - 0.5;
		v.y = frand() - 0.5;
		v.z = frand() - 0.5;
		m = v3dot(v, v);
	}while(m < 1e-4);
	return v3scale(v, r/sqrt(m));
}

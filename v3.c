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
 * Right-handed cross product, a x b: perpendicular to both a and
 * b, with the usual sign convention (v3cross(v3(1,0,0),v3(0,1,0))
 * == v3(0,0,1)).  Added for callers that build a screen-space
 * basis from an arbitrary eye/center/up triple rather than the
 * yaw/pitch orbit parameterization viewbasis() already covers.
 */
Vec3
v3cross(Vec3 a, Vec3 b)
{
	Vec3 v;

	v.x = a.y*b.z - a.z*b.y;
	v.y = a.z*b.x - a.x*b.z;
	v.z = a.x*b.y - a.y*b.x;
	return v;
}

/*
 * Unit vector in the direction of a.  No zero-length guard (a
 * zero-length input yields nan components); callers that might
 * pass a zero vector should check v3len() first, the same as any
 * hand-rolled normalize would have to.
 */
Vec3
v3unit(Vec3 a)
{
	double len;

	len = v3len(a);
	if(len < 1e-15)
		return v3(0, 0, 0);
	return v3scale(a, 1.0/len);
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

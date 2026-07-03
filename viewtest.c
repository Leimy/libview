#include <u.h>
#include <libc.h>
#include "view.h"

/*
 * viewtest - standalone exercise of the libview math, no draw.h
 * or event.h.  Prints one "PASS name" / "FAIL name: detail" line
 * per check and exits(nil) iff every check passed.  Deterministic:
 * srand(1) is called first so v3rand()-driven paths repeat.
 */

static int failed;

static void
ok(char *name)
{
	print("PASS %s\n", name);
}

static void
bad(char *name, char *fmt, ...)
{
	va_list arg;
	char buf[256];

	va_start(arg, fmt);
	vsnprint(buf, sizeof buf, fmt, arg);
	va_end(arg);
	print("FAIL %s: %s\n", name, buf);
	failed = 1;
}

/* reference viewbasis formulas, independent of the library's copy */
static void
refbasis(double yaw, double pitch, Vec3 *ex, Vec3 *ey, Vec3 *ez)
{
	double cla, sla, clo, slo;

	cla = cos(pitch * PI/180.0);
	sla = sin(pitch * PI/180.0);
	clo = cos(yaw * PI/180.0);
	slo = sin(yaw * PI/180.0);

	ex->x = -slo;      ex->y = clo;       ex->z = 0;
	ey->x = -sla*clo;  ey->y = -sla*slo;  ey->z = cla;
	ez->x = cla*clo;   ez->y = cla*slo;   ez->z = sla;
}

static double
adiff(Vec3 a, Vec3 b)
{
	return fabs(a.x-b.x) + fabs(a.y-b.y) + fabs(a.z-b.z);
}

static void
testbasis(void)
{
	static double yaws[] = { -180, -90, -35, 0, 45, 120, 179 };
	static double pits[] = { -90, -60, -18, 0, 18, 60, 90 };
	Vec3 ex, ey, ez, rx, ry, rz;
	double e, worst, w;
	int i, j;
	int orthook, unitok, exzok, refok;

	orthook = unitok = exzok = refok = 1;
	e = 1e-9;
	worst = 0.0;
	for(i = 0; i < nelem(yaws); i++)
		for(j = 0; j < nelem(pits); j++){
			viewbasis(yaws[i], pits[j], &ex, &ey, &ez);

			/* orthonormal: unit lengths */
			if(fabs(v3len(ex)-1.0) > 1e-9
			|| fabs(v3len(ey)-1.0) > 1e-9
			|| fabs(v3len(ez)-1.0) > 1e-9)
				unitok = 0;

			/* orthonormal: mutually orthogonal */
			w = fabs(v3dot(ex,ey)) + fabs(v3dot(ey,ez)) + fabs(v3dot(ex,ez));
			if(w > worst)
				worst = w;
			if(w > 1e-9)
				orthook = 0;

			/* ex lies in the z=0 plane (screen-right, no tilt) */
			if(fabs(ex.z) > 1e-15)
				exzok = 0;

			/* agrees with the reference formulas */
			refbasis(yaws[i], pits[j], &rx, &ry, &rz);
			if(adiff(ex,rx)+adiff(ey,ry)+adiff(ez,rz) > e)
				refok = 0;
		}

	if(unitok)
		ok("basis-unit");
	else
		bad("basis-unit", "a basis vector was not unit length");
	if(orthook)
		ok("basis-orthogonal");
	else
		bad("basis-orthogonal", "max off-orthogonality %g", worst);
	if(exzok)
		ok("basis-exz");
	else
		bad("basis-exz", "ex.z was nonzero");
	if(refok)
		ok("basis-reference");
	else
		bad("basis-reference", "disagreed with reference formulas");
}

static void
testnorm(void)
{
	Orbit o;

	/* yaw wrap: poke past +180 and past -180, orbitnorm wraps in */
	orbitinit(&o, 0, 0, 1);
	o.yaw = 200.0;
	orbitnorm(&o);
	if(o.yaw > -161.0 && o.yaw < -159.0)
		ok("orbitnorm-wrap-hi");
	else
		bad("orbitnorm-wrap-hi", "yaw=%g want -160", o.yaw);

	o.yaw = -200.0;
	orbitnorm(&o);
	if(o.yaw > 159.0 && o.yaw < 161.0)
		ok("orbitnorm-wrap-lo");
	else
		bad("orbitnorm-wrap-lo", "yaw=%g want 160", o.yaw);

	/* pitch clamp to [-90,90] default */
	o.pitch = 130.0;
	orbitnorm(&o);
	if(o.pitch == 90.0)
		ok("orbitnorm-pitch-hi");
	else
		bad("orbitnorm-pitch-hi", "pitch=%g want 90", o.pitch);

	o.pitch = -130.0;
	orbitnorm(&o);
	if(o.pitch == -90.0)
		ok("orbitnorm-pitch-lo");
	else
		bad("orbitnorm-pitch-lo", "pitch=%g want -90", o.pitch);

	/* zoom clamp to [0.1,128] default */
	o.zoom = 1000.0;
	orbitnorm(&o);
	if(o.zoom == 128.0)
		ok("orbitnorm-zoom-hi");
	else
		bad("orbitnorm-zoom-hi", "zoom=%g want 128", o.zoom);

	o.zoom = 0.001;
	orbitnorm(&o);
	if(o.zoom == 0.1)
		ok("orbitnorm-zoom-lo");
	else
		bad("orbitnorm-zoom-lo", "zoom=%g want 0.1", o.zoom);
}

/*
 * Drag across the +-180 seam should produce a small, wrapped
 * velocity, not ~360/dt.  Start yaw near +180; a small rightward
 * drag pushes yaw down through -180 (wraps to near +180); the
 * tracked vyaw must reflect the small step, not the raw ~360 jump.
 */
static void
testseam(void)
{
	Orbit o;
	double degperpx;

	orbitinit(&o, 179.0, 0, 1);
	degperpx = 0.35;
	orbitdown(&o, 0, 0, 1000);
	/* mouse moves left a few px: yaw = 179 - (-dx)*deg = increases,
	 * crossing +180 and wrapping to negative. dx = -20 -> +7 deg. */
	orbitmove(&o, -20, 0, 1020, degperpx);

	/* dt = 20ms, wrapped delta ~7 deg -> ~0.35 deg/ms, small */
	if(fabs(o.vyaw) < 1.0)
		ok("seam-velocity-small");
	else
		bad("seam-velocity-small", "vyaw=%g (raw ~360/dt bug?)", o.vyaw);

	/* sanity: it did wrap across the seam */
	if(o.yaw < 0.0)
		ok("seam-wrapped");
	else
		bad("seam-wrapped", "yaw=%g did not cross the seam", o.yaw);
}

/* grab (orbitdown) stops any existing spin dead */
static void
testgrab(void)
{
	Orbit o;

	orbitinit(&o, 0, 0, 1);
	o.vyaw = 0.5;
	o.vpitch = 0.3;
	orbitdown(&o, 10, 10, 500);
	if(o.vyaw == 0.0 && o.vpitch == 0.0)
		ok("grab-stops-spin");
	else
		bad("grab-stops-spin", "vyaw=%g vpitch=%g", o.vyaw, o.vpitch);
}

/*
 * orbittick: while dragging returns 0; at rest returns 0; a live
 * spin decays under friction and reaches full rest in a bounded
 * number of ticks (returning 0 thereafter).
 */
static void
testtick(void)
{
	Orbit o;
	int i, moved, hitrest;

	/* dragging -> no tick motion */
	orbitinit(&o, 0, 0, 1);
	o.vyaw = 0.5;
	orbitdown(&o, 0, 0, 0);
	if(orbittick(&o, 25.0) == 0)
		ok("tick-dragging-zero");
	else
		bad("tick-dragging-zero", "ticked while dragging");
	orbitup(&o);

	/* at rest -> no tick motion */
	orbitinit(&o, 0, 0, 1);
	if(orbittick(&o, 25.0) == 0)
		ok("tick-rest-zero");
	else
		bad("tick-rest-zero", "ticked at rest");

	/* live spin decays to rest within a bounded step count */
	orbitinit(&o, 0, 0, 1);
	o.vyaw = 0.2;
	o.vpitch = -0.1;
	hitrest = 0;
	for(i = 0; i < 100000; i++){
		moved = orbittick(&o, 25.0);
		if(!moved){
			hitrest = 1;
			break;
		}
	}
	if(hitrest && o.vyaw == 0.0 && o.vpitch == 0.0)
		ok("tick-friction-rest");
	else
		bad("tick-friction-rest", "no rest after %d ticks (vyaw=%g vpitch=%g)",
			i, o.vyaw, o.vpitch);
}

/*
 * springstep settles a small graph (5 bodies, 4 edges, defaults)
 * to below 0.0015 max speed within a bounded number of steps, and
 * never lets any body exceed the |pos| clamp.
 */
static void
testspring(void)
{
	enum { Nb = 5, Ne = 4, Maxstep = 20000 };
	Sbody body[Nb];
	Sbody *bp[Nb];
	Sedge edge[Ne];
	Spring s;
	double mv, worstpos, p;
	int i, k, settled, nstep;

	mv = 0.0;

	springdefaults(&s);
	memset(body, 0, sizeof body);
	for(i = 0; i < Nb; i++){
		body[i].pos = v3rand(1.5 + frand());
		bp[i] = &body[i];
	}
	/* a little star/chain: 0-1, 1-2, 2-3, 3-4 */
	for(i = 0; i < Ne; i++){
		edge[i].a = i;
		edge[i].b = i+1;
	}
	s.b = bp;
	s.nb = Nb;
	s.e = edge;
	s.ne = Ne;

	settled = 0;
	nstep = 0;
	worstpos = 0.0;
	for(i = 0; i < Maxstep; i++){
		mv = springstep(&s);
		nstep++;
		for(k = 0; k < Nb; k++){
			p = v3len(body[k].pos);
			if(p > worstpos)
				worstpos = p;
		}
		if(mv < 0.0015){
			settled = 1;
			break;
		}
	}

	if(settled)
		ok("spring-settles");
	else
		bad("spring-settles", "not below 0.0015 after %d steps (mv=%g)", nstep, mv);

	/* clamp is 5.0 by default; allow a hair of float slop */
	if(worstpos <= s.clamp + 1e-9)
		ok("spring-clamp");
	else
		bad("spring-clamp", "|pos|=%g exceeded clamp %g", worstpos, s.clamp);
}

void
main(void)
{
	srand(1);

	testbasis();
	testnorm();
	testseam();
	testgrab();
	testtick();
	testspring();

	if(failed)
		exits("fail");
	exits(nil);
}

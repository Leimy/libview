#include <u.h>
#include <libc.h>
#include "view.h"

/*
 * Orbit camera with inertial drag.  This is the single home for
 * the view-basis math and the drag/momentum state machine that
 * claudegraph (setview + main()'s button-1 code) and radioglobe
 * (viewbasis + main()'s Emouse/Etick code) used to each carry
 * their own copy of, including the wrap-safe velocity delta
 * across the +-180 seam and the grab-stops-spin behavior.
 */

/*
 * Screen-space basis for a view direction (yaw, pitch), in
 * degrees.  ex/ey are the screen-right/screen-up unit vectors,
 * ez points at the viewer.  Identical formulas to claudegraph's
 * setview() and radioglobe's viewbasis(), with yaw==lon and
 * pitch==lat.
 */
void
viewbasis(double yaw, double pitch, Vec3 *ex, Vec3 *ey, Vec3 *ez)
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

void
orbitinit(Orbit *o, double yaw, double pitch, double zoom)
{
	memset(o, 0, sizeof *o);
	o->yaw = yaw;
	o->pitch = pitch;
	o->zoom = zoom;
	o->pitchmin = -90.0;
	o->pitchmax = 90.0;
	o->zoommin = 0.1;
	o->zoommax = 128.0;
	o->friction = 0.92;
	o->vmin = 0.0005;
	/*
	 * frictionref stays 0 (legacy: one friction application per
	 * orbittick call) so existing callers are unaffected; see
	 * view.h.  A caller wanting real-time decay sets it to its
	 * nominal tick length after orbitinit.
	 */
	/* vyaw/vpitch, drag state all zeroed by memset */
	orbitnorm(o);
}

/*
 * Re-wrap yaw into [-180,180] and re-clamp pitch/zoom.  Called
 * after the application pokes fields directly (keyboard nudges),
 * and internally by orbitinit.
 */
void
orbitnorm(Orbit *o)
{
	while(o->yaw > 180.0)
		o->yaw -= 360.0;
	while(o->yaw < -180.0)
		o->yaw += 360.0;
	if(o->pitch > o->pitchmax)
		o->pitch = o->pitchmax;
	if(o->pitch < o->pitchmin)
		o->pitch = o->pitchmin;
	if(o->zoom > o->zoommax)
		o->zoom = o->zoommax;
	if(o->zoom < o->zoommin)
		o->zoom = o->zoommin;
}

/*
 * Button-down: start a drag and stop any spin dead, like
 * grabbing a spinning globe.
 */
void
orbitdown(Orbit *o, int x, int y, ulong msec)
{
	o->dragging = 1;
	o->startx = x;
	o->starty = y;
	o->startyaw = o->yaw;
	o->startpitch = o->pitch;
	o->vyaw = 0.0;
	o->vpitch = 0.0;
	o->lastyaw = o->yaw;
	o->lastpitch = o->pitch;
	o->lastmsec = msec;
}

/*
 * Mouse moved while the button is held.  yaw decreases as the
 * mouse moves right, pitch increases as it moves down (the sign
 * convention shared by both programs, must not flip).  Velocity
 * (deg/ms) is tracked only when msec advances past lastmsec,
 * wrapping the yaw delta first so the +-180 seam doesn't launch
 * the view at ~360/dt.
 */
void
orbitmove(Orbit *o, int x, int y, ulong msec, double degperpx)
{
	double dt, dy;

	if(!o->dragging)
		return;

	o->yaw = o->startyaw - (double)(x - o->startx)*degperpx;
	o->pitch = o->startpitch + (double)(y - o->starty)*degperpx;
	if(o->pitch > o->pitchmax)
		o->pitch = o->pitchmax;
	if(o->pitch < o->pitchmin)
		o->pitch = o->pitchmin;
	while(o->yaw > 180.0)
		o->yaw -= 360.0;
	while(o->yaw < -180.0)
		o->yaw += 360.0;

	if(msec > o->lastmsec){
		dt = msec - o->lastmsec;
		dy = o->yaw - o->lastyaw;
		while(dy > 180.0)
			dy -= 360.0;
		while(dy < -180.0)
			dy += 360.0;
		o->vyaw = dy / dt;
		o->vpitch = (o->pitch - o->lastpitch) / dt;
		o->lastyaw = o->yaw;
		o->lastpitch = o->pitch;
		o->lastmsec = msec;
	}
}

void
orbitup(Orbit *o)
{
	o->dragging = 0;
}

/*
 * One integration step of length ms with a single friction
 * application: the legacy orbittick body, factored out so
 * orbittick can repeat it when subdividing (see frictionref).
 */
static void
orbitstep(Orbit *o, double ms, double fric)
{
	o->yaw += o->vyaw * ms;
	o->pitch += o->vpitch * ms;
	if(o->pitch > o->pitchmax){
		o->pitch = o->pitchmax;
		o->vpitch = 0.0;
	}
	if(o->pitch < o->pitchmin){
		o->pitch = o->pitchmin;
		o->vpitch = 0.0;
	}
	while(o->yaw > 180.0)
		o->yaw -= 360.0;
	while(o->yaw < -180.0)
		o->yaw += 360.0;
	o->vyaw *= fric;
	o->vpitch *= fric;
	if(fabs(o->vyaw) < o->vmin && fabs(o->vpitch) < o->vmin){
		o->vyaw = 0.0;
		o->vpitch = 0.0;
	}
}

/*
 * Advance the inertial spin by ms milliseconds and apply
 * friction.  Returns 0 while dragging or when both velocities
 * are already zero (nothing to do); otherwise integrates,
 * wraps yaw / clamps pitch (a pitch clamp zeroes vpitch),
 * applies friction, zeroes both velocities once both fall below
 * vmin, and returns 1 (caller should redraw).
 *
 * With frictionref set (see view.h), ms is subdivided into steps
 * of at most frictionref so that the decay tracks elapsed time
 * instead of call count: an application whose ticks run late
 * (slow frames) still sees the spin cover the same ground in the
 * same wall-clock time.  ms == frictionref yields exactly one
 * step, identical to the legacy path; a trailing partial step
 * gets a correspondingly reduced friction, pow(friction,
 * dt/frictionref).
 */
int
orbittick(Orbit *o, double ms)
{
	double ref, dt;

	if(o->dragging)
		return 0;
	if(o->vyaw == 0.0 && o->vpitch == 0.0)
		return 0;
	if(ms <= 0.0)
		return 0;

	ref = o->frictionref;
	if(ref <= 0.0){
		/* legacy: one step, one friction application */
		orbitstep(o, ms, o->friction);
		return 1;
	}

	while(ms > 0.0){
		dt = ms > ref ? ref : ms;
		orbitstep(o, dt, dt == ref ? o->friction : pow(o->friction, dt/ref));
		ms -= dt;
		if(o->vyaw == 0.0 && o->vpitch == 0.0)
			break;		/* spin ended mid-interval */
	}
	return 1;
}

void
orbitzoom(Orbit *o, double factor)
{
	o->zoom *= factor;
	if(o->zoom > o->zoommax)
		o->zoom = o->zoommax;
	if(o->zoom < o->zoommin)
		o->zoom = o->zoommin;
}

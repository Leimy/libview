/*
 * libview - shared 3D view math for radioglobe and claudegraph.
 *
 * Pure math: no draw.h, no event.h, no locking, no I/O.  The
 * caller owns the event loop, the projection style (orthographic
 * or perspective), and any display locking; this library only
 * owns the numbers.
 *
 * Requires u.h and libc.h to be included first (double, ulong,
 * frand, sqrt, ...).  v3rand() uses frand(); seed with srand()
 * in the application if you care.
 *
 * This header is the API contract between the library and its
 * two consumers.  Do not change signatures casually; both
 * projects compile against it.
 */

typedef struct Vec3 Vec3;
struct Vec3 {
	double	x, y, z;
};

/* v3.c: vector helpers */
Vec3	v3(double x, double y, double z);
Vec3	v3add(Vec3, Vec3);
Vec3	v3sub(Vec3, Vec3);
Vec3	v3scale(Vec3, double);
double	v3dot(Vec3, Vec3);
double	v3len(Vec3);
Vec3	v3rand(double r);	/* uniform-direction vector of length r */
Vec3	v3cross(Vec3, Vec3);	/* right-handed cross product, a x b */
Vec3	v3unit(Vec3);		/* unit vector in the same direction; no zero-length guard, a zero vector yields nan */

/*
 * orbit.c: a camera orbiting the origin, with inertial drag.
 *
 * yaw/pitch are in degrees; radioglobe maps lon->yaw, lat->pitch.
 * viewbasis() computes the screen-space basis for a view
 * direction: ex/ey are the screen-right/screen-up unit vectors,
 * ez points at the viewer.  Projecting a world point is then
 * three dot products; what you do with them (orthographic disc,
 * perspective divide) is up to the application.
 */
void	viewbasis(double yaw, double pitch, Vec3 *ex, Vec3 *ey, Vec3 *ez);

typedef struct Orbit Orbit;
struct Orbit {
	/* view state; applications may poke these directly,
	 * then call orbitnorm() to re-clamp/re-wrap */
	double	yaw;		/* degrees, wrapped to [-180,180] */
	double	pitch;		/* degrees, clamped to [pitchmin,pitchmax] */
	double	zoom;

	/* limits (set after orbitinit if defaults don't suit) */
	double	pitchmin, pitchmax;
	double	zoommin, zoommax;

	/* momentum tuning */
	double	friction;	/* velocity decay factor per tick */
	double	vmin;		/* deg/ms below which spin stops */
	/*
	 * Reference tick length in ms for `friction`, enabling
	 * real-time (frame-rate-independent) decay.  0 (the
	 * orbitinit default) selects the legacy behavior: friction
	 * is applied exactly once per orbittick() call, whatever ms
	 * says, so the spin's trajectory depends on how often the
	 * application manages to call orbittick.  That is fine for
	 * a caller whose tick rate it can actually keep up with.
	 *
	 * Set it to the nominal tick length (e.g. 25.0 for a 40Hz
	 * timer) and orbittick will instead subdivide the ms it is
	 * given into steps of at most this length, so a call with
	 * 4x the nominal ms behaves exactly as four nominal calls
	 * would have.  Wall-clock spin-down then stays correct even
	 * when frames (and therefore ticks) are late or dropped:
	 * pass the real elapsed time since the previous call.
	 * Passing exactly frictionref reproduces the legacy path
	 * bit-for-bit, so switching a caller over is safe.
	 */
	double	frictionref;

	/* momentum state (read if curious; owned by the library) */
	double	vyaw, vpitch;	/* degrees per millisecond */

	/* drag state (internal) */
	int	dragging;
	int	startx, starty;
	double	startyaw, startpitch;
	double	lastyaw, lastpitch;
	ulong	lastmsec;
};

/*
 * orbitinit sets the given view plus defaults: pitch clamp
 * [-90,90], zoom clamp [0.1,128], friction 0.92, vmin 0.0005.
 *
 * Drag protocol: on button-down call orbitdown (this stops any
 * spin dead, like grabbing a spinning globe); on each mouse
 * event while the button is held call orbitmove with the
 * current position, event msec, and the drag sensitivity in
 * degrees per pixel (may vary per call, e.g. with zoom); on
 * release call orbitup.  Velocity is tracked during the drag
 * (with wrap-safe yaw deltas across the +-180 seam) so the
 * release inherits the launch speed.
 *
 * orbittick advances the inertial spin by ms milliseconds and
 * applies friction; it returns 1 if the view moved (caller
 * should redraw) and 0 if there is no spin (or a drag is in
 * progress).  Pitch hitting its clamp zeroes vpitch.  See
 * Orbit.frictionref above for making the decay depend on
 * elapsed time rather than on call count.
 *
 * orbitzoom multiplies zoom by factor and clamps.
 * orbitnorm re-wraps yaw and re-clamps pitch/zoom after the
 * application pokes fields directly (keyboard nudges etc.).
 */
void	orbitinit(Orbit*, double yaw, double pitch, double zoom);
void	orbitdown(Orbit*, int x, int y, ulong msec);
void	orbitmove(Orbit*, int x, int y, ulong msec, double degperpx);
void	orbitup(Orbit*);
int	orbittick(Orbit*, double ms);
void	orbitzoom(Orbit*, double factor);
void	orbitnorm(Orbit*);

/*
 * spring.c: force-directed graph layout (spring embedder).
 *
 * The application owns the node storage; each layout
 * participant embeds an Sbody (position, velocity, and a force
 * accumulator used as scratch during a step).  The Spring
 * describes one layout problem: an array of pointers to bodies,
 * an array of edges (index pairs into that array), and the
 * tuning parameters.
 *
 * springstep runs one step: gravity toward the origin (kcent),
 * pairwise repulsion (krep/d^2, O(n^2)), springs along edges
 * (kspring toward restlen), then integration with damping, a
 * per-step speed cap (maxv), and a hard clamp on |pos| (clamp;
 * 0 disables).  Coincident bodies are shoved apart in a random
 * direction.  Returns the largest body speed this step, so the
 * caller can stop ticking once the layout settles.
 *
 * springdefaults fills in the tuning that claudegraph shipped
 * with: restlen 1.3, kspring 0.10, krep 0.50, kcent 0.03,
 * damp 0.80, maxv 0.30, clamp 5.0.  It does not touch the
 * body/edge arrays.
 */
typedef struct Sbody Sbody;
struct Sbody {
	Vec3	pos;
	Vec3	vel;
	Vec3	frc;	/* scratch: force accumulator for one step */
};

typedef struct Sedge Sedge;
struct Sedge {
	int	a, b;	/* indices into the Spring's body array */
};

typedef struct Spring Spring;
struct Spring {
	Sbody	**b;
	int	nb;
	Sedge	*e;
	int	ne;

	double	restlen;	/* edge rest length */
	double	kspring;	/* edge spring constant */
	double	krep;		/* pairwise repulsion */
	double	kcent;		/* gravity toward the origin */
	double	damp;		/* velocity damping per step */
	double	maxv;		/* speed cap per step */
	double	clamp;		/* hard cap on |pos|; 0 = none */
};

void	springdefaults(Spring*);
double	springstep(Spring*);

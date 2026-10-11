# libview: shared 3D view library for radioglobe and claudegraph

## Goal

Extract the code duplicated (and already diverging) between
/usr/dave/work/radioglobe and /usr/dave/work/claude9/claudegraph.c
into one small, pure-math library, then adapt both programs to use
it.  Behavior of both programs must be preserved exactly.

The shared surface, established by side-by-side reading:

1. Vec3 math (claudegraph's vadd/vsub/vmul/vdot/rndvec; radioglobe's
   Vec3 + open-coded dot products).
2. The orbit-camera view basis (claudegraph setview() and radioglobe
   viewbasis() are the same formulas; yaw/pitch == lon/lat).
3. The inertial drag/momentum state machine (grab stops spin dead,
   deg/ms velocity tracking with wrap-safe deltas across the +-180
   seam, per-tick friction, vmin cutoff).  Copy-pasted once already;
   the antimeridian wrap fix exists in both copies and must live in
   exactly one place from now on.
4. The spring embedder (claudegraph physstep()).  Single consumer
   today; extracted for structure and testability, per
   claude9/claudegraph-review.md.

NOT shared, on purpose: projection style (orthographic vs
perspective), rendering, status bars, event loops, locking.  The
library has no draw.h/event.h dependency at all.

## API contract

/usr/dave/work/libview/view.h, written by the coordinator, is the
contract.  Implementers and adapters code against it.  Signature
changes require a stated reason in the agent's final summary.

## Build integration

- The library builds libview.a$O plus a viewtest binary in
  /usr/dave/work/libview via its own mkfile (</$objtype/mkfile,
  explicit ar rule).
- Each consumer references the archive by absolute path:
      LIBVIEW=/usr/dave/work/libview/libview.a$O
  adds -I/usr/dave/work/libview to CFLAGS, links $LIBVIEW into the
  affected binary only (claudegraph; radioglobe -- not claude9fs,
  mkcoast, mkstations), and carries a virtual rule so the library
  is rebuilt if stale:
      $LIBVIEW:V:
              @{cd /usr/dave/work/libview && mk}
  This work-tree-pointing setup is deliberate while all three
  projects are under active development together; see the
  README's "Linking it into a consumer" section.
- libview also has a "mk install" target, added once a real repo
  existed for it (matching the libao precedent): it copies
  libview.a$O to $home/lib and view.h to $home/include, flat, no
  per-objtype subdirectory (the $O suffix already disambiguates
  architecture).  Neither current consumer uses the installed
  copy yet -- that switch is for when libview lives in its own
  pushed repo and a consumer no longer shares this tree.

## Phases and ownership

Phase 1 (serial): agent A (opus) implements the library.
  Owns: /usr/dave/work/libview/* except view.h and PLAN.md.
  Writes: v3.c, orbit.c, spring.c, viewtest.c, mkfile, README.
  Must not touch the claude9 or radioglobe trees.

Phase 2 (parallel, after A is reviewed):
  Agent B (sonnet): adapts claudegraph.
    Owns: /usr/dave/work/claude9/claudegraph.c and
    /usr/dave/work/claude9/mkfile.  Must not touch claude.c,
    claude9fs.c, json.*, claudetalk*, skills/.
  Agent C (sonnet): adapts radioglobe.
    Owns: /usr/dave/work/radioglobe/{dat.h,globe.c,coast.c,main.c,
    mkfile} as needed.  Must not touch the claude9 tree or
    libview/.

The coordinator reviews after each phase and runs mk itself.

## Behavior fidelity table

Tuning that must survive the refactor unchanged:

  claudegraph: yaw -35, pitch 18, zoom 1; pitch clamp +-85;
    zoom 0.2..8 (keyboard steps 1.3x, wheel 1.15x); drag 0.35
    deg/px fixed; friction 0.92, vmin 0.0005; autorotate 0.12
    deg/tick stays app-side; spring params restlen 1.3, kspring
    0.10, krep 0.50, kcent 0.03, damp 0.80, maxv 0.30, world
    clamp 5.0, settle threshold Stopv 0.0015 (app-side compare
    against springstep's return).
  radioglobe: clat 30 -> pitch, clon 0 -> yaw, zoom 1; pitch
    clamp +-90; zoom 0.5..128; drag sensitivity 180.0/rad deg/px
    recomputed per event from globegeom (pass as orbitmove's
    degperpx); friction 0.92, vmin 0.0005; keyboard arrows nudge
    10.0/zoom degrees (poke fields + orbitnorm).

Drag sign conventions (identical in both programs today, must not
flip): yaw decreases as the mouse moves right (yaw = startyaw -
dx*degperpx), pitch increases as the mouse moves down (pitch =
startpitch + dy*degperpx).

The table above describes the state at refactor time and was met.
Divergence since, all consumer-side, none of it a fidelity
regression: radioglobe raised its own zoommax from 128 to 512 (to
separate same-city station clusters) and opted into
Orbit.frictionref = 25 so its spin decays in real time rather than
per orbittick() call -- its per-pixel textured-earth render can
make a frame outlast the tick interval.  claudegraph sets neither
and is bit-for-bit unaffected, which is exactly what
frictionref's 0 default guarantees.

## Testing

viewtest (agent A) is a standalone binary exercising the math with
no display: basis orthonormality at sample angles, yaw wrap and
pitch clamp in orbitnorm/orbittick, wrap-safe drag velocity across
the +-180 seam, grab-stops-spin, spring settling on a small graph
(max speed falls below 0.0015 within a bounded number of steps).
Prints one PASS/FAIL line per check; exits nil on all-pass.  The
coordinator only compiles it; the human runs it.

Added with frictionref: testfrictionref, asserting that 4 steps of
ref equal 1 step of 4*ref, that ms == frictionref reproduces the
legacy single step exactly, that the frictionref == 0 default still
moves 4x as far and decays once for a 4x call (the claudegraph
compatibility guarantee), and that a zero interval is a no-op.
Also testvec, covering v3cross/v3unit.  Run: `mk && ./viewtest`.

## Out of scope (tracked from claudegraph-review.md, not this job)

- Multiple claudegraph window instances guard.
- The implicit /srv naming contract between claudetalk and
  claudegraph.
- Snapshot-parser (parsegraph) tests.
- Man page for libview; system-wide install.

#include <u.h>
#include <libc.h>
#include "view.h"

/*
 * Force-directed graph layout (spring embedder), a faithful
 * port of claudegraph's physstep() over Sbody pointers and an
 * explicit Sedge list instead of Node.parentidx.  Same force
 * model, same integration order, same coincident-body random
 * shove, same speed cap and |pos| clamp, same return value
 * (the largest body speed this step, so the caller can tell
 * when the layout has settled).
 */

void
springdefaults(Spring *s)
{
	s->restlen = 1.3;
	s->kspring = 0.10;
	s->krep = 0.50;
	s->kcent = 0.03;
	s->damp = 0.80;
	s->maxv = 0.30;
	s->clamp = 5.0;
	/* body/edge arrays left untouched */
}

double
springstep(Spring *s)
{
	int i, j;
	Sbody *a, *b;
	Vec3 d;
	double d2, dist, mag, maxv, v;

	if(s->nb == 0)
		return 0.0;

	/* gravity toward the origin */
	for(i = 0; i < s->nb; i++)
		s->b[i]->frc = v3scale(s->b[i]->pos, -s->kcent);

	/* pairwise repulsion */
	for(i = 0; i < s->nb; i++)
		for(j = i+1; j < s->nb; j++){
			a = s->b[i];
			b = s->b[j];
			d = v3sub(a->pos, b->pos);
			d2 = v3dot(d, d);
			if(d2 < 1e-4){
				/* coincident: shove apart in a random direction */
				d = v3rand(0.05);
				d2 = v3dot(d, d);
			}
			dist = sqrt(d2);
			mag = s->krep / d2;
			a->frc = v3add(a->frc, v3scale(d, mag/dist));
			b->frc = v3sub(b->frc, v3scale(d, mag/dist));
		}

	/* springs along edges */
	for(i = 0; i < s->ne; i++){
		a = s->b[s->e[i].a];
		b = s->b[s->e[i].b];
		d = v3sub(b->pos, a->pos);
		dist = sqrt(v3dot(d, d));
		if(dist < 1e-3)
			continue;
		mag = s->kspring * (dist - s->restlen);
		a->frc = v3add(a->frc, v3scale(d, mag/dist));
		b->frc = v3sub(b->frc, v3scale(d, mag/dist));
	}

	/* integrate with damping, a speed cap, and a |pos| clamp */
	maxv = 0.0;
	for(i = 0; i < s->nb; i++){
		a = s->b[i];
		a->vel = v3scale(v3add(a->vel, a->frc), s->damp);
		v = sqrt(v3dot(a->vel, a->vel));
		if(v > s->maxv){
			a->vel = v3scale(a->vel, s->maxv/v);
			v = s->maxv;
		}
		a->pos = v3add(a->pos, a->vel);
		if(s->clamp > 0.0){
			d2 = v3dot(a->pos, a->pos);
			if(d2 > s->clamp*s->clamp)
				a->pos = v3scale(a->pos, s->clamp/sqrt(d2));
		}
		if(v > maxv)
			maxv = v;
	}
	return maxv;
}

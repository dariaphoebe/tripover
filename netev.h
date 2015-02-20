// condense.h - make network more dense

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

extern void ininetev(void);

extern ub4 prepestdur(lnet *net,ub4 *trip,ub4 len);
extern ub4 estdur(lnet *net,ub4 *trip1,ub4 len1,ub4 *trip2,ub4 len2);
extern ub4 estdur_3(lnet *net,ub4 h1,ub4 h2,ub4 h3);
extern ub4 estdur_2(lnet *net,ub4 h1,ub4 h2);
extern int mksubevs(lnet *net);
extern int rmsubevs(lnet *net);

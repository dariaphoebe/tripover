// event.h - time events

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

extern void inievent(int pass);

extern void showxtime(struct timepatbase *tp,ub8 *xp,ub4 xlim);

extern ub4 fillxtime(struct timepatbase *tp,ub8 *xp,ub1 *xpacc,ub4 xlen,ub4 gt0,struct sidbase *sp,ub1 *daymap,ub4 tdep,ub4 tid);
extern ub4 fillxtime2(struct timepatbase *tp,ub8 *xp,ub1 *xpacc,ub4 xlen,ub4 gt0,struct sidbase *sp,ub1 *daymap,ub4 tdep,ub4 tid,ub4 dur,ub4 srdep,ub4 srarr);
extern ub4 findtrep(struct timepatbase *tp,ub8 *xp,ub1 *xpacc,ub8 *xp2,ub4 xlim,ub4 evcnt);
extern ub4 filltrep(block *evmem,block *evmapmem,struct timepatbase *tp,ub8 *xp,ub1 *xpacc,ub4 xlim);
extern void clearxtime(struct timepatbase *tp,ub8 *xp,ub1 *xpacc,ub4 xlim,ub4 gt0);

// around 30 min
#define Accshift 5

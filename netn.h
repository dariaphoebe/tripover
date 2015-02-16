// netn.h - precompute n-stop connections

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */


extern void ininetn(void);
extern int mknet1(struct network *net,ub4 varlimit,ub4 var12limit,bool nilonly);
extern int mknet2(struct network *net,ub4 varlimit,ub4 var12limit,bool nilonly);
extern int mknetn(struct network *net,ub4 nstop,ub4 varlimit,ub4 var12limit,bool nilonly);

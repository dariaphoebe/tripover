// fare.h - handle fare/seat availiability for reserved transport

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

extern void inifare(void);
extern int fareupd(gnet *net,ub4 rid,ub4 hop1,ub4 hop2,ub4 chop,ub4 t,ub4 mask,ub4 nfare,ub4 *fares);

// condense.h - make network more dense

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

extern void ininetev(void);

extern ub4 prepestdur(net *net,ub4 *trip,ub4 len,ub4 *estdurs,ub4 estdurcnt);
extern ub4 estdur(net *net,ub4 *estdurs,ub4 estdurcnt,ub4 *trip1,ub4 len1,ub4 *trip2,ub4 len2);

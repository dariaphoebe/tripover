// math.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef M_PI
  #define M_PI 3.141592655
#endif

struct range {
  ub4 lo,hi,hilo;
  ub4 lopos,hipos;
};

extern int minmax(ub4 *x, ub4 n, struct range *rp);
extern int minmax2(ub2 *x, ub4 n, struct range *rp);
extern int mkhist(ub4 *data, ub4 n,struct range *rp, ub4 ivcnt,ub4 *bins, const char *desc,ub4 lvl);
extern int mkhist2(ub2 *data, ub4 n,struct range *rp, ub4 ivcnt,ub4 *bins, const char *desc,ub4 lvl);

extern ub4 rnd(ub4 range);
extern double frnd(ub4 range);

extern int mkheightmap(ub4 *map,ub4 n);

extern double lat2rad(ub4 lat);
extern double lon2rad(ub4 lon);
extern ub4 rad2lat(double rlat);
extern ub4 rad2lon(double rlon);

extern double geodist(double rlat1, double rlon1, double rlat2, double rlon2);

extern int inimath(void);
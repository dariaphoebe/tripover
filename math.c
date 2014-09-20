// math.c - math utilities

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* math utilities like statistics, geo, random
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "base.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "mem.h"
#include "util.h"
#include "time.h"

int minmax(ub4 *x, ub4 n, struct range *rp)
{
  ub4 lo = hi32;
  ub4 hi = 0;
  ub4 lopos = 0;
  ub4 hipos = 0;
  ub4 i,v;

  for (i = 0; i < n; i++) {
    v = x[i];
    if (v < lo) { lo = v; lopos = i; }
    if (v > hi) { hi = v; hipos = i; }
  }
  rp->hi = hi; rp->lo = lo;
  rp->hipos = hipos; rp->lopos = lopos;
  rp->hilo = hi - lo;
  info(0,"lo %u hi \ah%u range %u",lo,hi,hi - lo);
  return (n == 0);
}

int minmax2(ub2 *x, ub4 n, struct range *rp)
{
  ub4 lo = hi32;
  ub4 hi = 0;
  ub4 lopos = 0;
  ub4 hipos = 0;
  ub4 i,v;

  for (i = 0; i < n; i++) {
    v = x[i];
    if (v < lo) { lo = v; lopos = i; }
    if (v > hi) { hi = v; hipos = i; }
  }
  rp->hi = hi; rp->lo = lo;
  rp->hipos = hipos; rp->lopos = lopos;
  rp->hilo = hi - lo;
  info(0,"lo %u hi \ah%u range %u",lo,hi,hi - lo);
  return (n == 0);
}

int mkhist(ub4 *data, ub4 n,struct range *rp, ub4 ivcnt,ub4 *bins, const char *desc,enum Msglvl lvl)
{
  ub4 lo,hi,i,v,iv;

  error_lt(ivcnt,2);
  error_z(n,0);

  minmax(data,n,rp);
  lo = rp->lo;
  hi = rp->hi;

  if (hi == lo) return info(0,"nil range for histogram %s", desc);

  memset(bins,0,ivcnt * sizeof(ub4));
  for (i = 0; i < n; i++) {
    v = data[i];
    if (v < lo) bins[ivcnt - 2]++;
    else if (v > hi) bins[ivcnt-1]++;
    else {
      iv = (v - lo) * (ivcnt - 2) / (hi - lo);
      error_ge(iv,ivcnt);
      bins[iv]++;
    }
  }
  for (iv = 0; iv < ivcnt; iv++) genmsg(lvl,User,"%s  bin %u: %u", desc,iv,bins[iv]);
  return 0;
}

int mkhist2(ub2 *data, ub4 n,struct range *rp, ub4 ivcnt,ub4 *bins, const char *desc,enum Msglvl lvl)
{
  ub4 lo,hi,i,v,iv,cnt,ccnt,sum,csum;

  error_lt(ivcnt,2);
  error_z(n,0);

  minmax2(data,n,rp);
  lo = rp->lo;
  hi = rp->hi;

  if (hi == lo) return info(0,"nil range for histogram %s", desc);

  memset(bins,0,ivcnt * sizeof(ub4));
  sum = 0;
  for (i = 0; i < n; i++) {
    v = data[i];
    sum += v;
    if (v < lo) bins[ivcnt - 2]++;
    else if (v > hi) bins[ivcnt-1]++;
    else {
      iv = (v - lo) * (ivcnt - 2) / (hi - lo);
      error_ge(iv,ivcnt);
      bins[iv]++;
    }
  }
  info(User,"%s: %u bins avg %u sum %u", desc,ivcnt,sum / n,sum);
  csum = 0;
  for (iv = 0; iv < ivcnt; iv++) csum += bins[iv];

  ccnt = 0;
  for (iv = 0; iv < ivcnt; iv++) {
    cnt = bins[iv];
    ccnt += cnt;
    genmsg(lvl,User,"  bin %u: %u %u%%", iv,cnt,ccnt * 100 / csum);
  }
  return 0;
}

// wikipedia xorshift
static ub8 xorshift64star(void)
{
  static ub8 x = 0x05a3ae52de3bbf0aULL;

  x ^= x >> 12; // a
  x ^= x << 25; // b
  x ^= x >> 27; // c
  return x * 2685821657736338717ULL;
}

static ub8 rndstate[ 16 ];

static ub4 xorshift1024star(void)
{
  static int p;

  ub8 s0 = rndstate[p];
  ub8 s1 = rndstate[p = ( p + 1 ) & 15];
  s1 ^= s1 << 31; // a
  s1 ^= s1 >> 11; // b
  s0 ^= s0 >> 30; // c
  rndstate[p] = s0 ^ s1;
  
  return (ub4)(rndstate[p] * 1181783497276652981ULL);
}

static ub4 rndmask(ub4 mask) { return (ub4)xorshift1024star() & mask; }

ub4 rnd(ub4 range) { return (ub4)((xorshift1024star() % range)); }

double frnd(ub4 range)
{
  double x;

  x = rnd(range);
  return x;
}

// diamond-square fractal landscape, after wikipedia and its links
int mkheightmap(ub4 *map,ub4 n)
{
  ub4 x,y,range,range1,len,len2;
  ub4 val00,val01,val10,val11,mval,cval0,cval1,cval2,cval3;

  len = n;
  len2 = len >> 1;
  range = (1 << 14);
  range1 = range - 1;

// seed 4 corners
/*
  map[0] = val0 + rnd(range1);
  map[n-1] = val0 + rnd(range1);
  map[n] = val0 + rnd(range1);
  map[n * n - 1] = val0 + rnd(range1);
*/
  do {
    if (range > 2) {
      range >>= 1;
      range1 = range - 1;
    }

    info(0,"len %u range %u",len,range);
    for (y = 0; y + len < n; y += len) {

      for (x = 0; x + len < n; x += len) {

        // diamond step
        val00 = map[y * n + x];
        val01 = map[(y + len) * n + x];
        val10 = map[y * n + x + len];
        val11 = map[(y + len) * n + x + len];
        mval = (val00 + val01 + val10 + val11) >> 2;
        mval += rndmask(range1);
        map[(y+len2) * n + x + len2] = mval;

        // square step
        cval0 = (val00 + val10 + mval) / 3;
        cval0 += rndmask(range1);
        map[(y + len2) * n + x] = cval0;

        cval1 = (val00 + val01 + mval) / 3;
        cval1 += rndmask(range1);
        map[y * n + x + len2] = cval1;

        cval2 = (val01 + val11 + mval) / 3;
        cval2 += rndmask(range1);
        map[(y + len2) * n + x + len] = cval2;

        cval3 = (val10 + val11 + mval) / 3;
        cval3 += rndmask(range1);
        map[(y + len) * n + x + len2] = cval3;
      }
    }
    len >>= 1;
    len2 >>= 1;
  } while(len);

  writeppm("heightmap.ppm",map,n,n);

  return 0;
}

// lat,lon to distance functions and vars
static double mean_earth_radius = 6371.0;

double lat2rad(ub4 lat) { return ((double)lat / Latscale - 90.0) * M_PI / 180.0; }
double lon2rad(ub4 lon) { return ((double)lon / Lonscale - 180.0) * 2 * M_PI / 360.0; }

ub4 rad2lat(double rlat) { return (ub4)(( (rlat * 180 / M_PI) + 90) * Latscale); }
ub4 rad2lon(double rlon) { return (ub4)(( (rlon * 180 / M_PI) + 180) * Lonscale); }

static double geolow = 6.0 / 40.000;

// great circle lat/lon to Km.
double geodist(double rlat1, double rlon1, double rlat2, double rlon2)
{
  double fdist, dlat, dlon;
  double a,b,c,phi1,phi2,lam1,lam2,dphi,dlam,dsig,dist;

  phi1 = rlat1;
  phi2 = rlat2;
  lam1 = rlon1;
  lam2 = rlon2;

  dlam = lam2 - lam1;
  dphi = phi2 - phi1;

  if (dlam > -geolow && dlam < geolow && dphi > -geolow && dphi < geolow) { // approx trivial case
//    vrb(0,"geodist trivial %e %e between |%e|",dlam,dphi,geolow);
    dlat = dlam * mean_earth_radius / 4 * M_PI;
    dlon = dphi * mean_earth_radius / 4 * M_PI;
    fdist = sqrt(dlat * dlat + dlon * dlon);
    return fdist;
  }

  // haversine functions
  a = sin(dphi * 0.5);
  a *= a;
  b = dlam * 0.5;
  b *= b;
  c = a + cos(phi1) * cos(phi2) * b;
  dsig = 2 * asin(sqrt(c));

  dist = dsig * mean_earth_radius;

  return dist;
}

int inimath(void)
{
  ub8 x;

  msgfile = setmsgfile(__FILE__);
  iniassert();

  for (x = 0; x < 16; x++) rndstate[x] = xorshift64star();

  return 0;
}
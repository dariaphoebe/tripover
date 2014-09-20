// netbase.c - base network with primary data

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Functions to setup or create a base network */

#include <string.h>

#include "base.h"
#include "mem.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "util.h"
#include "bitfields.h"
#include "netbase.h"

static netbase basenet;

netbase *getnetbase(void) { return &basenet; }

void ininetbase(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

// check whether ports are in trip
// todo use portsbyhop
int portintrip(ub4 *legs,ub4 nleg,ub4 dep,ub4 mid,ub4 arr)
{
  ub4 legno,leg,hopcnt = basenet.hopcnt;
  struct hopbase *hp,*hops = basenet.hops;

  if (nleg == 0) return 0;

  leg = legs[0];
  if (leg >= hopcnt) return 1;

  hp = hops + leg;

  if (nleg == 1) {
    if (hp->dep == mid) return 2;
    if (hp->dep == arr) return 3;
    if (hp->arr == dep) return 4;
    if (hp->arr == mid) return 5;
  }

  if (hp->dep == mid) return 6;
  if (hp->dep == arr) return 7;
  if (hp->arr == dep) return 8;
  if (hp->arr == mid) return 9;

//  if (hp->arr == arr) return 10;

  for (legno = 1; legno < nleg-1; legno++) {
    leg = legs[legno];
    if (leg >= hopcnt) return 1;
    hp = hops + leg;
    if (hp->dep == dep) return 11;
    if (hp->dep == mid) return 12;
    if (hp->dep == arr) return 13;
    if (hp->arr == dep) return 14;
    if (hp->arr == mid) return 15;
    if (hp->arr == arr) return 16;
  }
  leg = legs[nleg-1];
  if (leg >= hopcnt) return 1;
  hp = hops + leg;
//  if (hp->dep == dep) return 17;
  if (hp->dep == mid) return 18;
  if (hp->dep == arr) return 19;
  if (hp->arr == dep) return 20;
  if (hp->arr == mid) return 21;

  return 0;
}

// check whether a triplet passes thru the given ports
void checktrip_fln(ub4 *legs, ub4 nleg,ub4 dep,ub4 arr,ub4 dist,ub4 fln)
{
  ub4 legno,legno2,leg,leg2,arr0,cdist,hopcnt = basenet.hopcnt;
  struct hopbase *hp,*hp2,*hops = basenet.hops;

  error_eq_fln(arr,dep,"arr","dep",fln);

  for (legno = 0; legno < nleg; legno++) {
    leg = legs[legno];
    error_ge_fln(leg,hopcnt,"hop","hopcnt",fln);
  }
  if (nleg > 2) {
    for (legno = 0; legno < nleg; legno++) {
      leg = legs[legno];
      hp = hops + leg;
      for (legno2 = legno+1; legno2 < nleg; legno2++) {
        leg2 = legs[legno2];
        hp2 = hops + leg2;
        error_eq_fln(leg,leg2,"leg1","leg2",fln);
        error_eq_fln(hp->dep,hp2->dep,"dep1","dep2",fln);
        error_eq_fln(hp->dep,hp2->arr,"dep1","arr2",fln);
        if (legno2 != legno + 1) error_eq_fln(hp->arr,hp2->dep,"arr1","dep2",fln);
        error_eq_fln(hp->arr,hp2->arr,"arr1","arr2",fln);
      }
    }
  }

  leg = legs[0];
  hp = hops + leg;
  error_ne_fln(hp->dep,dep,"hop.dep","dep",fln);
  arr0 = hp->arr;
  cdist = hp->dist;

  leg = legs[nleg-1];
  hp = hops + leg;
  error_ne_fln(hp->arr,arr,"hop.arr","arr",fln);

  for (legno = 1; legno < nleg; legno++) {
    leg = legs[legno];
    hp = hops + leg;
    error_ne_fln(arr0,hp->dep,"prv.arr","dep",fln);
    arr0 = hp->arr;
    cdist += hp->dist;
  }
  error_ne_fln(dist,cdist,"dist","cdist",fln);
}

// check whether a triplet passes thru the given ports
void checktrip3_fln(ub4 *legs, ub4 nleg,ub4 dep,ub4 arr,ub4 via,ub4 dist,ub4 fln)
{
  ub4 legno,leg;
  struct hopbase *hp,*hops = basenet.hops;
  int hasvia = 0;

  error_lt_fln(nleg,2,"nleg","2",fln);
  checktrip_fln(legs,nleg,dep,arr,dist,fln);
  error_eq_fln(via,dep,"via","dep",fln);
  error_eq_fln(via,arr,"via","arr",fln);

  for (legno = 1; legno < nleg; legno++) {
    leg = legs[legno];
    hp = hops + leg;
    if (hp->dep == via) hasvia = 1;
  }
  error_z_fln(hasvia,0,"via","0",fln);
}

static void addport(struct portbase *ports,ub4 port,ub4 lat,ub4 lon,double rlat,double rlon,ub4 size,enum txkind kind)
{
  struct portbase *pp = ports + port;

  pp->lat = lat;
  pp->lon = lon;
  pp->rlat = rlat;
  pp->rlon = rlon;
  if (kind == Air) pp->air = 1;
  else if (kind == Rail) pp->rail = 1;
  pp->id = port;
  pp->size = size;
}

/* generate artificial network for testing purposes
 for realism, ports are randomly placed on a fractal landscape 
 ports get a random size, used to bias the #connections
 no clustering yet
*/
#define Zmap (2048+1)
#define Zhist (256+2)

static ub4 heightmap[Zmap * Zmap];

static ub4 getz(ub4 lat,ub4 lon)
{
  ub4 y = lat * Zmap / (180 * Latscale);
  ub4 x = lon * Zmap / (360 * Lonscale);

  error_ge(y,Zmap);
  error_ge(x,Zmap);
  return heightmap[y * Zmap + x];
}

int mkrandnet(ub4 portcnt,ub4 hopcnt)
{
  struct portbase *ports,*pdep,*parr;
  struct hopbase *hops;
  struct range zrange;
  ub1 *net0;
  ub4 hist[Zhist];
  ub4 iv,cnt,waterlvl,z,lat,lon,railcnt,iter,aircnt,aimed,landrange;
  double rlat,rlon;
  ub4 dep,arr,curhop,depcnt;
  ub4 depstats[16];

  if (portcnt == 0 || hopcnt == 0) return 1;

  info(0,"generate artificial %u port %u hop network",portcnt,hopcnt);

  ports = alloc(portcnt,struct portbase,0,"baseports",portcnt);
  hops = alloc(hopcnt,struct hopbase,0,"basehops",hopcnt);
  net0 = alloc(portcnt * portcnt, ub1,0,"net0",portcnt);

  // fractal land to create net on
  mkheightmap(heightmap,Zmap);
  mkhist(heightmap,Zmap,&zrange,Zhist,hist,"randnet",Vrb);

  iv = 0; cnt = 0;
  while (iv < Zhist && cnt < Zmap * 70 / 100) {
    cnt += hist[iv++];
  }
  if (iv) iv--;
  waterlvl = zrange.lo + zrange.hilo * iv / (Zhist - 2);
  info(0,"70%% = %u before bin %u", waterlvl, iv);
  landrange = zrange.hi - waterlvl;

  aimed = portcnt / 4;

  iter = railcnt = 0;
  while (railcnt < aimed && iter++ < (1 << 20)) {
    rlat = (frnd(1600) - 900) * 0.1 * M_PI / 180;
    rlon = (frnd(3600) - 1800) * 0.1 * M_PI / 180;
    lat = rad2lat(rlat);
    lon = rad2lon(rlon);
    vrb(0,"port %u: lat %u lon %u rlat %e rlon %e",railcnt,lat,lon,rlat,rlon);
    z = getz(lat,lon);
    if (z < waterlvl) continue;
    if (z - waterlvl < rnd(landrange)) {
      addport(ports,railcnt++,lat,lon,rlat,rlon,rnd(100),Rail);
    }
  }

  iter = aircnt = 0;
  while (railcnt + aircnt < portcnt && iter++ < (1 << 20)) {
    rlat = (frnd(1600) - 900) * 0.1 * M_PI / 180;
    rlon = (frnd(3600) - 1800) * 0.1 * M_PI / 180;
    lat = rad2lat(rlat);
    lon = rad2lon(rlon);
    vrb(0,"port %u: lat %u lon %u rlat %e rlon %e",railcnt+aircnt,lat,lon,rlat,rlon);
    z = getz(lat,lon);
    if (z < waterlvl) continue;
    if (z - waterlvl < rnd(landrange)) {
      addport(ports,railcnt + aircnt,lat,lon,rlat,rlon,rnd(100),Air);
      aircnt++;
    }
  }
  basenet.ports = ports;

  curhop = 0;
  iter = 0;
  while (curhop < hopcnt && iter++ < (1 << 20)) {
    dep = rnd(portcnt);
    arr = rnd(portcnt);
    if (dep == arr) continue;

    error_ge(dep,portcnt);
    error_ge(arr,portcnt);

    if (rnd(100) > ports[dep].size) continue;

    net0[dep * portcnt + arr] = 1;
    hops[curhop].dep = dep;
    hops[curhop].arr = arr;
    curhop++;
    pdep = ports + dep;
    parr = ports + arr;
    pdep->deps++;
    parr->arrs++;
  }

  aclear(depstats);
  for (dep = 0; dep < portcnt; dep++) {
    depcnt = 0;
    for (arr = 0; arr < portcnt; arr++) {
      depcnt += net0[dep * portcnt + arr];
    }
    depstats[min(Elemcnt(depstats),depcnt)]++;
  }
  for (iv = 0; iv < Elemcnt(depstats); iv++) vrb(0,"%u ports with %u departures", depstats[iv], iv);

  basenet.hops = hops;
  basenet.portcnt = portcnt;
  basenet.hopcnt = hopcnt;

//  net2txt(&basenet);

  info0(0,"done generating artificial net");
  return 0;
}

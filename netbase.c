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
  struct portbase *ports;
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
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);

    if (rnd(100) > ports[dep].size) continue;

    net0[dep * portcnt + arr] = 1;
    hops[curhop].dep = dep;
    hops[curhop].arr = arr;
    curhop++;
  }

  aclear(depstats);
  for (dep = 0; dep < portcnt; dep++) {
    depcnt = 0;
    for (arr = 0; arr < portcnt; arr++) {
      depcnt += net0[dep * portcnt + arr];
    }
    depstats[min(Elemcnt(depstats),depcnt)]++;
  }
  for (iv = 0; iv < Elemcnt(depstats); iv++) info(0,"%u ports with %u departures", depstats[iv], iv);

  basenet.hops = hops;
  basenet.portcnt = portcnt;
  basenet.hopcnt = hopcnt;
  info0(0,"done generating artificial net");
  return 0;
}

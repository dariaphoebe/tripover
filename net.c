// net.c - main network setup

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Initialize the network :

   - Separate ports in full and minor.
     Full ports have enough connectivity to include in a (full x full) matrix.
     Minor ports connect only to one or two single transfer stops.
     Actual plannig is done on these transfer ports.

   - Build connectivity matrix between any 2 full ports
     base matrix for direct (non-stop) hops
     derived matrix for each of n intermediate hops

   - Prepare various metrics used for heuristics
 */

#include <string.h>

#include "base.h"
#include "mem.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "util.h"
#include "bitfields.h"
#include "netbase.h"
#include "net.h"

static struct network net;

void ininet(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

#define Geohist (100+2)

// create 0-stop connectivity matrix and derived info
static int mknet0(void)
{
  netbase *basenet = net.base;
  ub4 portcnt = net.portcnt;
  ub4 hopcnt = net.hopcnt;

  struct portbase *bports,*dport,*aport;
  struct port *ports;
  struct hopbase *hp,*bhops;
  struct range distrange;
  ub2 concnt,*con0cnt;
  ub4 ofs,*con0ofs;
  ub4 hop,*con0lst;
  ub2 dist,*dist0;
  ub4 geohist[Geohist];
  double fdist;
  ub4 dep,arr,port2,da,depcnt;
  ub2 iv;
  ub4 depstats[16];

  if (portcnt == 0 || hopcnt == 0) return 1;

  info(0,"0-stop connections for %u port %u hop network",portcnt,hopcnt);

  port2 = portcnt * portcnt;
  bports = basenet->ports;
  bhops = basenet->hops;

  ports = net.ports;

  con0cnt = alloc(port2, ub2,0,"con0cnt",portcnt);
  con0ofs = alloc(port2, ub4,0xff,"con0ofs",portcnt);
  con0lst = alloc(hopcnt, ub4,0,"con0lst",hopcnt);

  // geographical direct-line distance
  dist0 = alloc(port2, ub2,0,"geodist0",portcnt);

  info(0,"calculating \ah%u distance pairs", port2);
  for (dep = 0; dep < portcnt; dep++) {
    dport = bports + dep;
    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;
      aport = bports + arr;
      fdist = geodist(dport->rlat,dport->rlon,aport->rlat,aport->rlon);
      dist = (ub2)fdist;
      dist0[dep * portcnt + arr] = dist;
    }
  }
  info(0,"done calculating \ah%u distance pairs", port2);
  mkhist2(dist0,port2,&distrange,Geohist,geohist,"geodist",Vrb);

  // create 0-stop connectivity
  // support multiple hops per port pair
  for (hop = 0; hop < hopcnt; hop++) {
    hp = bhops + hop;
    dep = hp->dep;
    arr = hp->arr;
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    da = dep * portcnt + arr;
    concnt = con0cnt[da];
    error_ovf(concnt,ub2);
    con0cnt[da] = (ub2)(concnt+1);
  }

  ofs = 0;
  for (dep = 0; dep < portcnt; dep++) {
    for (arr = 0; arr < portcnt; arr++) {
      da = dep * portcnt + arr;
      concnt = con0cnt[da];
      if (concnt == 0) continue;

      con0ofs[da] = ofs;
      for (hop = 0; hop < hopcnt; hop++) {
        hp = bhops + hop;
        if (hp->dep != dep || hp->arr != arr) continue;
        error_ge(ofs,hopcnt);
        con0lst[ofs++] = hop;
      }
    }
  }

  // get connectivity stats
  aclear(depstats);
  for (dep = 0; dep < portcnt; dep++) {
    depcnt = 0;
    for (arr = 0; arr < portcnt; arr++) {
      depcnt += con0cnt[dep * portcnt + arr];
      error_ovf(depcnt,ub2);
    }
    ports[dep].depcnt = (ub2)depcnt;
    depstats[min(Elemcnt(depstats),depcnt)]++;
  }
  for (iv = 0; iv < Elemcnt(depstats); iv++) info(0,"%u ports with %u departures", depstats[iv], iv);

  net.con0cnt = con0cnt;
  net.con0ofs = con0ofs;
  net.con0lst = con0lst;

  return 0;
}

int mknet(netbase *basenet)
{
  ub4 port,portcnt = basenet->portcnt;
  ub4 hop,hopcnt = basenet->hopcnt;
  struct port *pp,*ports;
  struct hop *hp,*hops;
  struct portbase *portbase = basenet->ports;
  struct hopbase *hopbase = basenet->hops;

  if (portcnt == 0 || hopcnt == 0) return 1;

  ports = alloc(portcnt,struct port,0,"ports",portcnt);
  hops = alloc(hopcnt,struct hop,0,"hops",hopcnt);

  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    pp->id = port;
    pp->base = portbase + port;
  }
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    hp->id = hop;
    hp->base = hopbase + hop;
  }

  net.portcnt = portcnt;
  net.hopcnt = hopcnt;
  net.ports = ports;
  net.hops = hops;
  net.base = basenet;

  if (mknet0()) return 1;
  return 0;
}

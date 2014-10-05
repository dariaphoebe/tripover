// netprep.c - prepare net from base

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <string.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "util.h"
#include "bitfields.h"
#include "netbase.h"
#include "netio.h"
#include "netprep.h"
#include "net.h"

void ininetprep(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

int prepnet(netbase *basenet)
{
  struct network *net = getnet();
  struct portbase *bports,*bpp;
  struct hopbase *bhops,*bhp;
  struct port *ports,*pdep,*parr,*pp;
  struct hop *hops,*hp;
  ub4 portcnt,hopcnt,dep,arr,ndep,narr,nodep,noarr,nodeparr,nudep,nuarr;
  ub4 nloc,nlen,n;
  enum txkind kind;
  ub4 hop,port;

  hopcnt = basenet->hopcnt;
  portcnt = basenet->portcnt;
  if (portcnt == 0 || hopcnt == 0) return 1;

  bports = basenet->ports;
  bhops = basenet->hops;

  ports = alloc(portcnt,struct port,0,"ports",portcnt);
  hops = alloc(hopcnt,struct hop,0,"hops",hopcnt);

  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    bpp = bports + port;
    pp->id = pp->allid = port;
    nlen = bpp->namelen;
    if (nlen) {
      memcpy(pp->name,bpp->name,nlen);
      pp->namelen = nlen;
    }
    pp->lat = bpp->lat;
    pp->lon = bpp->lon;
    pp->rlat = bpp->rlat;
    pp->rlon = bpp->rlon;
    pp->utcofs = bpp->utcofs;
  }

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    hp->id = hop;
    bhp = bhops + hop;
    nlen = bhp->namelen;
    if (nlen) {
      memcpy(hp->name,bhp->name,nlen);
      hp->namelen = nlen;
    }
    dep = bhp->dep;
    arr = bhp->arr;
    hp->dep = dep;
    hp->arr = arr;
    pdep = ports + dep;
    parr = ports + arr;

    kind = bhp->kind;
    switch(kind) {
    case Walk: hp->walk = 1; pdep->nwalkdep++; parr->nwalkarr++; break;
    case Air: hp->air = 1; break;
    case Rail: hp->rail = 1; break;
    case Bus: hp->bus = 1; break;
    case Unknown: info(0,"hop %s has unknown transport mode", hp->name); break;
    }

  }

  net->allportcnt = portcnt;
  net->allhopcnt = hopcnt;
  net->allports = ports;
  net->allhops = hops;

// mark local links

  nloc = Elemcnt(pdep->deps);

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;

    dep = hp->dep;
    arr = hp->arr;
    error_ge(dep,portcnt);
    error_ge(arr,portcnt);
    pdep = ports + dep;
    parr = ports + arr;
    ndep = pdep->ndep;
    narr = parr->narr;

    pdep->ndep = ndep+1;
    parr->narr = narr+1;

    if (hp->walk) continue;

    nudep = pdep->nudep;
    nuarr = parr->nuarr;
    switch(nudep) {
    case 0: pdep->deps[0] = arr; pdep->nudep = 1; break;
    case 1: if (pdep->deps[0] != arr) { pdep->deps[1] = arr; pdep->nudep = 2; } break;
    case 2: if (pdep->deps[0] != arr && pdep->deps[1] != arr) pdep->nudep = 3; break;
    default: pdep->nudep = nudep + 1; break; // tentative
    }
    switch(nuarr) {
    case 0: parr->arrs[0] = dep; parr->nuarr = 1; break;
    case 1: if (parr->arrs[0] != dep) { parr->arrs[1] = dep; parr->nuarr = 2; } break;
    case 2: if (parr->arrs[0] != dep && parr->arrs[1] != dep) parr->nuarr = 3; break;
    default: parr->nuarr = nuarr + 1; break; // tentative
    }
  }
  nodep = noarr = nodeparr = 0;

  ub4 constats[256];
  aclear(constats);

  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    ndep = pp->ndep; narr = pp->narr;
    if (ndep == 0 && narr == 0) { info(0,"port %u has no connections - %s",port,pp->name); nodeparr++; }
    else if (ndep == 0) { info(0,"port %u has no deps - %s",port,pp->name); nodep++; }
    else if (narr == 0) { info(0,"port %u has no arrs - %s",port,pp->name); noarr++; }
    if (ndep < 16 && narr < 16) constats[(ndep << 4) | narr]++;
  }
  info(0,"%u of %u ports without departures",nodep,portcnt);
  info(0,"%u of %u ports without arrivals",noarr,portcnt);
  info(0,"%u of %u ports without connection",nodeparr,portcnt);
  for (ndep = 0; ndep < 4; ndep++) {
    for (narr = 0; narr < 4; narr++) {
      n = constats[(ndep << 4) | narr];
      if (n || ndep < 2 || narr < 2) info(0,"%u ports with %u deps + %u arrs", n,ndep,narr);
    }
  }
  return 0;
}

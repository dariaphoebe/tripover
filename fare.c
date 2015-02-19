// fare.c - handle fare/seat availiability for reserved transport

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* reserved transport - like long-distance trains and planes -  have fares available on
  an individual trip basis. Whether reserved is determined on a per-route basis.
  If so, a list of fare positions per fare group per expanded departure is maintained.
  The entries contain the price. This list is accessed in parallel to time events.
 */

#include <string.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "math.h"

static ub4 msgfile;
#include "msg.h"

#include "util.h"
#include "time.h"
#include "net.h"
#include "fare.h"

#undef hdrstop

int fareupd(gnet *net,ub4 rid,ub4 hop1,ub4 hop2,ub4 chop,ub4 t,ub4 mask,ub4 nfare,ub4 *fares)
{
  ub4 *fhopofs = net->fhopofs;
  struct timepat *tp;
  struct hop *hp,*hops = net->hops;
  ub4 f,gndx,ofs;
  ub2 *farepos,*fareposbase = net->fareposbase;
  block *faremem = &net->faremem;
  ub8 *ev,*events = net->events;
  ub4 rt,tt,prvt;

  hp = hops + hop1;
  tp = &hp->tp;

  info(0,"rid %u hop2 %u",rid,hop2);
  if (nfare > Faregrp) return error(0,"farecnt %u above %u",nfare,Faregrp);

  ub4 evcnt = tp->evcnt;

  ub4 gt0 = tp->gt0;
  
  if (evcnt == 0) return info(0,"no events for %u",hop1);
  ev = events + tp->evofs;

  ofs = fhopofs[chop];

  farepos = fareposbase + ofs * Faregrp;
  bound(faremem,(ofs + nfare) * Faregrp,ub2);

  gndx = 0; tt = prvt = 0;
  while (gndx < evcnt) {
    prvt = tt;
    rt = (ub4)ev[gndx * 2];
    tt = rt + gt0;
    if (tt >= t) break;
    gndx++;
  }
  if (gndx == evcnt) return info(0,"\ad%u after \ad%u",t,tt);

  if (gndx && tt > t) {
    info(0,"found \ad%u before \ad%u",t,tt);
    gndx--; rt = (ub4)ev[gndx * 2]; tt = rt + gt0;
  }
  info(0,"found \ad%u as \ad%u after \ad%u",t,tt,prvt);

  bound(faremem,(ofs + gndx + (Faregrp-1)) * Faregrp,ub2);

  farepos += gndx * Faregrp;

  if (mask & 0xf0) {
    mask >>= 4; f = 0;
    while (mask) {
      farepos[f] = hi16;
      info(0,"fare %u set to n/a",f);
    }
    mask >>= 1;
  }
  for (f = 0; f < nfare; f++) {
    farepos[f] = (ub2)fares[f];
    info(0,"fare %u set to %u",f,fares[f]);
  }

  return 0;
}

void inifare(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

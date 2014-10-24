// search.c

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* search

  currently a rudimentary routing-only search.

 */

#include <string.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "time.h"
#include "util.h"

static ub4 msgfile;
#include "msg.h"

#include "bitfields.h"
#include "net.h"

#include "search.h"

void inisearch(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

static int srcgeo(struct network *net,ub4 stop,search *src)
{
  ub2 *cnts = net->concnt[stop];
  ub4 *ofss = net->conofs[stop];
  ub4 *lst;
  block *lstblk = &net->conlst[stop];
  ub4 *lodists = net->lodist[stop];
  ub4 portcnt = net->portcnt;
  ub4 dep = src->mdep;
  ub4 arr = src->marr;
  ub4 nleg = stop + 1;

  ub4 cnt,cost,dist,lodist,leg,l,ofs;
  ub4 da = dep * portcnt + arr;
  ub4 *vp;

  ub4 costlim = src->costlim;
  ub4 *hopdist = net->hopdist;
  ub4 v0 = 0;

  cnt = cnts[da];
  if (cnt == 0) return info(0,"no %u-stop connection %u-%u",stop,dep,arr);
  ofs = ofss[da];
  lst = blkdata(lstblk,0,ub4);
  if (ofs >= net->lstlen[stop]) return error(0,"offset %u out of %lu range",ofs,net->lstlen[stop]);
  vp = lst + ofs * nleg;
  cost = hi32;

  lodist = src->lodist;
  error_z(lodist,stop);

  do {
    l = vp[0];
    dist = hopdist[l];
    for (leg = 1; leg < nleg; leg++) {
      l = vp[leg];
      dist += hopdist[l];
    }
    if (dist < lodist) {
      error_z(dist,v0);
      lodist = dist;
      src->lostop = stop;
      memcpy(src->trip,vp,nleg * sizeof(ub4));
      if (dist == lodists[da]) info(0,"%u-stop found lodist %u at var %u",stop,dist,v0);
    }
    v0++;
    vp += nleg;
  } while (v0 < cnt && cost > costlim);
  src->tripcnt = 1;
  src->lodist = lodist;
  return 0;
}

// rudimentary geographic aka spatial only search
// no time or cost
int searchgeo(search *src,ub4 dep,ub4 arr,ub4 nstoplo,ub4 nstophi)
{
  ub4 mdep,marr;
  ub4 stop,nleg;
  int rv;
  struct network *net = getnet(0);
  ub4 portcnt = net->allportcnt;
  ub4 maxstop = net->maxstop;
  ub4 *mac2port = net->mac2port;
  struct port *parr,*pdep,*allports = net->allports;

  if (dep >= portcnt) return error(0,"departure %u not in %u portlist",dep,portcnt);
  if (arr >= portcnt) return error(0,"arrival %u not in %u portlist",arr,portcnt);

  if (nstophi > maxstop) { warning(0,"limiting max %u-stop to %u", nstophi,maxstop); nstophi = maxstop; }
  if (nstoplo > nstophi) { warning(0,"setting min %u-stop to %u", nstoplo,nstophi); nstoplo = nstophi; }

  info(0,"search geo dep %u arr %u between %u and %u stops",dep,arr,nstoplo,nstophi);

  clear(src);

  src->dep = dep;
  src->arr = arr;

  pdep = allports + dep;
  parr = allports + arr;

  // find macro port in case of mini
  if (pdep->mini) {
    mdep = mac2port[pdep->macid];
  } else mdep = dep;
  src->mdep = mdep;

  if (parr->mini) {
    marr = mac2port[parr->macid];
  } else marr = arr;
  src->marr = marr;

  src->costlim = 0;
  src->lodist = hi32;

  for (stop = 0; stop <= nstophi; stop++) {
    info(CC,"search geo dep %u arr %u in %u-stop",dep,arr,stop);
    rv = srcgeo(net,stop,src);
    if (rv) return rv;
  }
  if (src->tripcnt == 0) return 0;
  nleg = src->lostop + 1;

  triptoports(net,src->trip,nleg,src->tripports);

  // todo: if dep != mdep or arr != marr add distance difference and allport to result

  return 0;
}

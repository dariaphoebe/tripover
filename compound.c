// compound.h - create compound hops from routes

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* add virtual non-stop hops between any two ports on the same route.
 a route is defined here as in public transport : ports are visited in succession
 without actual tranfers.

 at network read time, these routes are ignored, yet the ID is marked.
 after condense, reconstruct the 'direct' hops that do not need a physical transfer
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
#include "net.h"
#include "compound.h"

#undef hdrstop

static ub2 maxroutelen = 100;

static ub4 rid2log = 0x3fa0;

void inicompound(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

// add compound hops
int compound(struct network *net)
{
  ub4 part = net->part;
  ub4 port,rportcnt,port2,portcnt = net->portcnt;
  ub4 hop,newhop,newhopcnt,addhopcnt,hopcnt = net->hopcnt;
  struct port *parr,*pdep,*ports = net->ports;
  struct hop *hp,*nhp,*newhops,*hops = net->hops;
  struct route *routes,*rp;
  enum txkind kind;
  ub4 dep,arr,ndep,narr,lodep,loarr,hidep,hiarr;
  ub2 rdep,rarr,rport;
  ub4 pos,n;
  ub4 rid,rrid,maxrid,rid4max,cnt,maxcnt,routecnt,routesum;
  ub2 cnt2;
  int docompound;
  enum Msglvl lvl = Vrb;

  if (portcnt == 0) return info0(0,"skip compound on 0 ports");
  if (hopcnt == 0) return info0(0,"skip compound on 0 hops");
  port2 = portcnt * portcnt;

  docompound = dorun(Runcompound);

  if (hopcnt > 1000 * 1000) return info(0,"no compound for \ah%u hops",hopcnt);

  maxrid = net->maxrouteid;

  if (maxrid > 1000 * 1000 * 100) return info(0,"no compound for max route ID \ah%u",maxrid);

  if (docompound == 0) {
    info(0,"no compound for %u ports",portcnt);
    return 0;
  }

  info(0,"compounding %u ports %u hops",portcnt,hopcnt);

  ub2 *rrid2hopcnt = alloc(maxrid+1,ub2,0,"cmp rrid2hopcnt",maxrid);
  ub4 *rrid2rid = alloc(maxrid+1,ub4,0,"cmp rrid2rid",maxrid);

  // count #hops per route
  maxcnt = rid4max = 0;
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    rrid = hp->routeid;
    if (rrid == hi32) continue;

    error_ne(hp->part,part);

    error_gt(rrid,maxrid);
    dep = hp->dep;
    arr = hp->arr;
    error_eq(dep,arr);

    cnt2 = rrid2hopcnt[rrid];
    cnt2++;
    limit(cnt2,maxroutelen,rrid);
    if (cnt2 > maxcnt) { maxcnt = cnt2; rid4max = rrid; }
    rrid2hopcnt[rrid] = cnt2;
  }

  // estimate #compound hops
  routecnt = routesum = addhopcnt = rid = 0;
  for (rrid = 0; rrid <= maxrid; rrid++) {
    cnt = rrid2hopcnt[rrid];
    if (cnt == 0) continue;
    rrid2rid[rrid] = rid++;
    routesum += cnt;
    if (cnt > 2) addhopcnt += cnt * (cnt - 1);
  }
  routecnt = rid;

  info(0,"%u hops in %u routes, max len %u at rid %x",routesum,routecnt,maxcnt,rid4max);
  info(0,"\ah%u tentative new hops",addhopcnt);

  ub2 *rid2hopcnt = alloc(routecnt,ub2,0,"rid2hopcnt",routecnt);
  routes = alloc(routecnt,struct route,0,"routes",routecnt);

  ub2 *port2rport = alloc(portcnt,ub2,0,"port2rport",portcnt);
  ub4 *rport2port = alloc(portcnt,ub4,0,"rport2port",portcnt);

  ub2 *portdeps = alloc(portcnt,ub2,0,"portdeps",portcnt);
  ub2 *portarrs = alloc(portcnt,ub2,0,"portarrs",portcnt);
  ub2 *rportdeps = alloc(portcnt,ub2,0,"rportdeps",portcnt);
  ub2 *rportarrs = alloc(portcnt,ub2,0,"rportarrs",portcnt);

  ub1 *rduphops = alloc(port2,ub1,0,"cmp duphops",port2);

  // resequence
  for (rrid = 0; rrid <= maxrid; rrid++) {
    cnt2 = rrid2hopcnt[rrid];
    if (cnt2 == 0) continue;
    rid = rrid2rid[rrid];
    rp = routes + rid;
    rp->routeid = rrid;
    rp->hopcnt = cnt2;
    rid2hopcnt[rid] = cnt2;
  }

  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    rrid = hp->routeid;
    if (rrid != hi32) hp->rid = rrid2rid[rrid];
    else hp->rid = hi32;
  }

  newhopcnt = hopcnt + addhopcnt;
  newhops = alloc(newhopcnt,struct hop,0,"cmp newhops",newhopcnt);

  memcpy(newhops,hops,hopcnt * sizeof(struct hop));

  ub4 *rhp,*ridhops = alloc(routecnt * maxcnt,ub4,0,"cmp ridhops",maxcnt);

  newhop = hopcnt;
  nhp = newhops + newhop;

  // fill hops per route
  for (rid = 0; rid < routecnt; rid++) {
    cnt = rid2hopcnt[rid];
    if (cnt < 3) continue;
    rhp = ridhops + rid * maxcnt;
    pos = 0;
    for (hop = 0; hop < hopcnt; hop++) {
      hp = hops + hop;
      if (hp->rid == rid) rhp[pos++] = hop;
    }
  }

  ub2 *rdp = alloc(maxcnt,ub2,0,"cmp reseq dep",maxcnt);
  ub2 *rap = alloc(maxcnt,ub2,0,"cmp reseq arr",maxcnt);
  ub2 *rdp2 = alloc(maxcnt,ub2,0,"cmp reseq dep",maxcnt);
  ub2 *rap2 = alloc(maxcnt,ub2,0,"cmp reseq arr",maxcnt);
  ub2 *spp = alloc(maxcnt * 2,ub2,0,"cmp reseq port",maxcnt * 2);

  ub4 spos,spos1;
  ub4 termroutes = 0;
  ub4 constats[256];

  // reconstruct routes
  for (rid = 0; rid < routecnt; rid++) {
    rp = routes + rid;
    rrid = rp->routeid;
    lvl = (rrid == rid2log ? Info : Vrb);
    cnt = rid2hopcnt[rid];
    if (cnt == 0) { info(0,"no hops for route %u %x",rid,rrid); continue; }
    else if (cnt < 2) { info(0,"skipping %u hop route %u %x",cnt,rid,rrid); continue; }

    rhp = ridhops + rid * maxcnt;
    memset(portdeps,0,portcnt * sizeof(ub2));
    memset(portarrs,0,portcnt * sizeof(ub2));
    hp = hops + rhp[0];
    rp->kind = kind = hp->kind;
    for (pos = 0; pos < cnt; pos++) {
      hop = rhp[pos];
      hp = hops + hop;
      dep = hp->dep;
      arr = hp->arr;
      pdep = ports + dep;
      parr = ports + arr;
      genmsg(lvl,0,"route %x hop %u %u-%u %s to %s",rrid,hop,dep,arr,pdep->name,parr->name);
      portdeps[dep]++;
      portarrs[arr]++;
    }

    // resequence ports per route
    rport = 0;
    lodep = loarr = hi32; hidep = hiarr = 0;
    for (port = 0; port < portcnt; port++) {
      ndep = portdeps[port];
      narr = portarrs[port];
      if (ndep == 0 && narr == 0) continue;
      lodep = min(lodep,ndep); loarr = min(loarr,narr);
      hidep = max(hidep,ndep); hiarr = max(hiarr,narr);
      pdep = ports + port;
      genmsg(lvl,0,"port %u at #%u: %u deps %u arrs %s",port,rport,ndep,narr,pdep->name);
      port2rport[port] = rport;
      rport2port[rport] = port;
      rport++;
    }
    rportcnt = rp->portcnt = rport;
    if (rportcnt < 3) { info(0,"skip %u-port route %u %x",rportcnt,rid,rrid); continue; }
    genmsg(lvl,0,"route %x %u ports deps %u-%u arrs %u-%u",rrid,rport,lodep,hidep,loarr,hiarr);

    memset(rportdeps,0,rportcnt * sizeof(ub2));
    memset(rportarrs,0,rportcnt * sizeof(ub2));

    // filter out variants
    for (pos = 0; pos < cnt; pos++) {
      hop = rhp[pos];
      hp = hops + hop;
      dep = hp->dep;
      arr = hp->arr;
      rdep = port2rport[dep];
      rarr = port2rport[arr];
      rportdeps[rdep]++;
      rportarrs[rarr]++;
    }

    for (pos = 0; pos < cnt; pos++) {
      hop = rhp[pos];
      hp = hops + hop;
      dep = hp->dep;
      arr = hp->arr;
      rdep = port2rport[dep];
      rarr = port2rport[arr];
      pdep = ports + dep;
      parr = ports + arr;
      if (rportdeps[rdep] > 1 && rportarrs[rarr] > 1) {
        info(0,"route %x omit %u-%u %s to %s",rrid,dep,arr,pdep->name,parr->name);
        rportdeps[rdep]--;
        rportarrs[rarr]--;
        rhp[pos] = hi32;
      }
    }

    memset(rduphops,0,rportcnt * rportcnt);

    // detect terminus
    aclear(constats);
    rp->dtermport = rp->atermport = hi32;
    for (rport = 0; rport < rportcnt; rport++) {
      port = rport2port[rport];
      pdep = ports + port;
      ndep = rportdeps[rport];
      narr = rportarrs[rport];

      if (ndep < 4 && narr < 4) constats[ (ndep << 4) | narr]++;
      else constats[0x44]++;

      if (ndep == 0 && narr == 0) {
        warning(0,"route %x port %u is not connected",rrid,port);
      } else if (rp->dtermport == hi32 && ndep && narr == loarr && loarr != hiarr) {
        genmsg(lvl,0,"route %x dep term port %u %s with %u deps %u arrs",rrid,port,pdep->name,ndep,narr);
        rp->dtermport = port;
      } else if (rp->atermport == hi32 && narr && ndep == lodep && lodep != hidep) {
        genmsg(lvl,0,"route %x arr term port %u %s with %u deps %u arrs",rrid,port,pdep->name,ndep,narr);
        rp->atermport = port;
      } else if (rp->dtermport != hi32 && ndep && narr == loarr && loarr != hiarr) {
        genmsg(lvl,0,"route %x extra dep term port %u %s with %u deps %u arrs",rrid,port,pdep->name,ndep,narr);
      } else if (rp->atermport != hi32 && narr && ndep == lodep && lodep != hidep) {
        genmsg(lvl,0,"route %x extra arr term port %u %s with %u deps %u arrs",rrid,port,pdep->name,ndep,narr);
      } else if (ndep > 3 || narr > 3) info(0,"route %x port %u with %u deps %u arrs",rrid,port,ndep,narr);
      else genmsg(lvl,0,"route %x port %u with %u deps %u arrs %s",rrid,port,ndep,narr,pdep->name);
    }
    if (rp->dtermport != hi32) termroutes++;
    else info(0,"route %x has no dep terms", rrid);

    for (ndep = 0; ndep < 3; ndep++) {
      for (narr = 0; narr < 3; narr++) {
        n = constats[(ndep << 4) | narr];
        if (n) genmsg(lvl,0,"%u port\as with %u deps + %u arrs", n,ndep,narr);
      }
    }

    // resequence route 1
    memset(rportdeps,0,rportcnt * sizeof(ub2));
    memset(rportarrs,0,rportcnt * sizeof(ub2));
    for (pos = 0; pos < cnt; pos++) {
      hop = rhp[pos];
      if (hop == hi32) continue;
      hp = hops + hop;
      dep = hp->dep;
      arr = hp->arr;
      rdep = port2rport[dep];
      rarr = port2rport[arr];
      error_ge(rdep,rportcnt);
      error_ge(rarr,rportcnt);
      rduphops[rdep * rportcnt + rarr] = 1;
      ndep = rportdeps[rdep];
      narr = rportarrs[rarr];
      if (ndep && narr) {
        rdep = rarr = hi16; // only store if dep and arr unique
        pdep = ports + dep; parr = ports + arr;
        info(0,"skip extra %u-%u %s to %s",dep,arr,pdep->name,parr->name);
      } else {
        rportdeps[rdep] = (ub2)ndep + 1;
        rportarrs[rarr] = (ub2)narr + 1;
      }
      rdp[pos] = rdp2[pos] = rdep;
      rap[pos] = rap2[pos] = rarr;
    }

    // start with departure terminus
    port = rp->dtermport;
    if (port == hi32) {
      warning(0,"route %u.%x nil dterm",part,rrid);
      continue; // later
    }
    if (port == rp->atermport) {
      pdep = ports + port;
      warning(0,"route %u.%x dterm = aterm port %u.%u %s",part,rrid,pdep->part,port,pdep->name);
    }

    pdep = ports + port;
    rport = port2rport[port];

    // resequence route 2
    spos = pos = 0;
    while (spos < rportcnt - 1) {
      pos = 0;
      if (port == rp->atermport) {
        warning(0,"route %u.%x arr %u into term port %u.%u %s",part,rrid,port,pdep->partcnt,pdep->part,pdep->name);
        break;
      }
      while (pos < cnt && rdp[pos] != rport) pos++;
      if (pos == cnt) {
        warning(0,"route %u.%x port %u.%u.%u %s not found",part,rrid,pdep->partcnt,pdep->part,port,pdep->name);
        break;
      }
      genmsg(lvl,0,"found %u at pos %u, seq %u %s",port,pos,spos,pdep->name);
      spp[spos++] = rport;
      rdp[pos] = hi16;
      rport = rap[pos];
      port = rport2port[rport];
      pdep = ports + port;
    }
    if (pos == cnt) continue;

    port = rp->atermport;
    if (port != hi32 && spos < rportcnt) {
      parr = ports + port;
      rport = port2rport[port];
      pos = 0;
      while (pos < cnt && rap[pos] != rport) pos++;
      if (pos == cnt) {
        warning(0,"route %x arr term port %u %s not found",rrid,port,parr->name);
        rportcnt = spos;
      } else {
        genmsg(lvl,0,"found %u at pos %u, seq %u %s",port,pos,spos,pdep->name);
        spp[spos++] = rport;
      }
    }

    // generate compounds
    for (spos = 0; spos < rportcnt; spos++) {
      rdep = spp[spos];
      dep = rport2port[rdep];
      for (spos1 = spos+2; spos1 < rportcnt; spos1++) {
        rarr = spp[spos1];
        if (rdep == rarr) continue;
        if (rduphops[rdep * rportcnt + rarr]) continue;

        rduphops[rdep * rportcnt + rarr] = 1;
        arr = rport2port[rarr];
        pos = 0;
        while (pos < cnt && !(rdp2[pos] == rdep && rap2[pos] == rarr) ) pos++; // skip existing
        if (pos < cnt) continue;

        error_ge(newhop,newhopcnt);

        pdep = ports + dep;
        parr = ports + arr;
        genmsg(lvl,0,"add compound %u-%u %s to %s",dep,arr,pdep->name,parr->name);
        error_ge(newhop,newhopcnt);
        nhp->dep = dep;
        nhp->arr = arr;
        nhp->kind = kind;
        nhp->rid = rid;
        nhp->routeid = rrid;
        nhp->compound = spos1 - spos;
//          fmtstring(nhp->name,"%s to %s",port->name,port1->name);
        nhp++;
        newhop++;
      }
    }

  } // each route
  info(0,"%u of %u routes with terminus",termroutes,routecnt);
  info(0,"%u of %u estimated compound hops",newhop - net->hopcnt,newhopcnt);
  newhopcnt = newhop;

  for (rid = 0; rid < routecnt; rid++) {
    cnt = rid2hopcnt[rid];
    if (cnt < 3) continue;
    rp = routes + rid;
    rrid = rp->routeid;
    rhp = ridhops + rid * maxcnt;
  }

  net->routecnt = routecnt;

  net->hopcnt = newhopcnt;
  net->hops = newhops;

  return 0;
}

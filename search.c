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

static void inisrc(search *src)
{
  ub4 i;

  src->costlim = 0;
  src->lodist = hi32;

  src->tripcnt = src->triplen = 0;
  for (i = 0; i < Nxleg; i++) {
    src->trip[i] = hi32;
    src->tripparts[i] = hi16;
    src->tripports[i] = hi32;
  }
}

static int merge(ub4 callee,search *s,search *d,char *ref)
{
  ub4 slen = s->triplen;
  ub4 sleg,dleg,l;

#if 0
  error_z(len1,0);
  error_z(len2,0);
  error_z(s1->tripcnt,0);
  error_z(s2->tripcnt,0);
#endif

  if (d->tripcnt == 0) d->tripcnt = 1;

  dleg = d->triplen;
  if (dleg >= Nxleg) return warning(0,"search %s exceeds max trip len %u for %u",ref,dleg,callee & hi16);
  for (sleg = 0; sleg < slen; sleg++) {
    if (dleg >= Nxleg) return warning(0,"search %s exceeds max trip len %u",ref,dleg);
    d->trip[dleg] = s->trip[sleg];
    d->tripparts[dleg] = s->tripparts[sleg];
    dleg++;
  }
  d->triplen = dleg;
  return 0;
}

// within single part
static ub4 srclocal(ub4 callee,struct network *net,ub4 part,ub4 dep,ub4 arr,ub4 stop,search *src,const char *desc)
{
  ub2 *cnts,cnt;
  ub4 *ofss,ofs;
  ub4 *lst;
  block *lstblk;
  ub4 *lodists,lodist;
  ub4 nleg = stop + 1;
  ub4 histop = net->histop;

  ub4 cost,dist = 0,leg,l;
  ub4 *vp;

  ub4 costlim = src->costlim;
  ub4 *hopdist;
  ub4 v0 = 0;

  ub4 portcnt = net->portcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 ln = callee & 0xffff;

  if (stop > histop) { vrb0(0,"%s: net part %u only has %u-stop connections, skip %u",desc,part,histop,stop); return 0; }

  cnts = net->concnt[stop];
  ofss = net->conofs[stop];
  lstblk = &net->conlst[stop];
  lodists = net->lodist[stop];
  hopdist = net->hopdist;

  ub4 da = dep * portcnt + arr;

  cnt = cnts[da];
  if (cnt == 0) { vrb0(0,"no %u-stop connection %u-%u",stop,dep,arr); return 0; }
  ofs = ofss[da];
  lst = blkdata(lstblk,0,ub4);
  error_ge(ofs,net->lstlen[stop]);
  vp = lst + ofs * nleg;
  cost = hi32;

  lodist = src->lodist;
  if (lodist == 0) lodist = hi32;

  do {

    // distance-only
    for (leg = 0; leg < nleg; leg++) {
      l = vp[leg];
      error_ge(l,chopcnt);
      dist += hopdist[l];
    }
    infovrb(dist == 0,0,"dist %u for var %u",dist,v0);
    if (dist < lodist) {
      if (dist == 0) warning(0,"dist 0 for len %u",nleg);
      lodist = dist;
      src->lostop = stop;
      memcpy(src->trip,vp,nleg * sizeof(ub4));
      for (leg = 0; leg < nleg; leg++) src->tripparts[leg] = (ub2)part;
      if (dist && dist == lodists[da]) info(0,"%u-stop found lodist %u at var %u %s:%u",stop,dist,v0,desc,ln);
    }
    // time
    // dt = datime(vp,nleg)
    v0++;
    vp += nleg;
  } while (v0 < cnt && cost > costlim);

  src->triplen = nleg;
  src->tripcnt = 1;

  src->lodist = lodist;
//  src->part = part;
  info(0,"%s: found %u-stop conn %u-%u",desc,stop,dep,arr);

  return 1;
}

// within single part
static ub4 srcglocal(struct gnetwork *gnet,ub4 part,ub4 gdep,ub4 garr,ub4 stop,search *src)
{
  ub4 dep,arr;
  struct network *net = getnet(part);
  ub4 portcnt = net->portcnt;
  ub4 gportcnt = gnet->portcnt;

  error_ge(gdep,gportcnt);

  dep = net->g2pport[gdep];
  arr = net->g2pport[garr];
  error_ge(dep,portcnt);
  error_ge(arr,portcnt);
  return srclocal(caller,net,part,dep,arr,stop,src,"local");
}

// search from top to arr
static ub4 srcxapart(struct gnetwork *gnet,ub4 gtdep,ub4 garr,ub4 nstop,ub4 dstop,search *asrc,search *tsrc,char *ref)
{
  struct network *net,*tnet,*anet;
  ub1 *portparts = gnet->portparts;
  ub4 partcnt = gnet->partcnt;
  ub4 dep,arr;
  ub4 part,apart;
  ub4 astop,tstop,dlostop,dhistop;
  ub4 xconn = 0;
  ub4 tpart;
  ub4 ti,ati,tcnt,tacnt;
  ub4 dtmid,atmid,gatmid,xamid,xmid;
  ub4 portcnt,tportcnt,aportcnt;
  ub4 stats[4];
  int rv;

  tpart = partcnt - 1;
  tnet = getnet(tpart);
  tportcnt = tnet->portcnt;

  dlostop = 0; dhistop = nstop - 1;

  info(0,"xapart gtdep %u",gtdep);

  for (apart = 0; apart < partcnt && xconn == 0; apart++) {
    if (portparts[garr * partcnt + apart] == 0) continue;
    anet = getnet(apart);
    aportcnt = anet->portcnt;
    arr = anet->g2pport[garr];
    error_ge(arr,aportcnt);
    tacnt = anet->tportcnt;
    for (ati = 0; ati < tacnt; ati++) {
      atmid = anet->tports[ati];

      for (astop = 0; astop < nstop - dstop; astop++) {
        rv = srclocal(caller,anet,apart,atmid,arr,astop,asrc,"xpart top-arr");
        if (rv == 0) continue;

        // arr.part AP has via TA in top

        // now check if top connects TD and TA
        gatmid = anet->p2gport[atmid];

        info(0,"part %u garr %u conn with gmid %u",apart,garr,gatmid);

        if (gtdep == gatmid) { stats[1]++; xconn++; break; }  // TA == TD

        xmid = tnet->g2pport[gtdep];
        error_ge(xmid,tportcnt);
        xamid = tnet->g2pport[gatmid];
        error_ge(xamid,tportcnt);

        for (tstop = 0; tstop < nstop - dstop; tstop++) {
          rv = srclocal(caller,tnet,tpart,xmid,xamid,tstop,tsrc,"xpart top");
          if (rv == 0) continue;
        }
      }
    }
  } // each arr.part
  return xconn;
}

static ub4 srcxpart(struct gnetwork *gnet,ub4 gdep,ub4 garr,ub4 nstop,search *src,char *ref)
{
  struct network *net,*tnet,*anet;
  search xsrc,tsrc,asrc;
  ub1 *portparts = gnet->portparts;
  ub4 partcnt = gnet->partcnt;
  ub4 dep,arr;
  ub4 part,apart;
  ub4 dstop,astop,tstop,dlostop,dhistop;
  ub4 conn,xconn = 0;
  ub4 tpart;
  ub4 ti,ati,tcnt,tacnt;
  ub4 dtmid,gdtmid,atmid,gatmid,xamid,xmid;
  ub4 portcnt,tportcnt,aportcnt;
  ub4 stats[4];
  int rv;

  if (partcnt == 1) { error(0,"interpart search called without partitions, ref %s",ref); return 0; }

  tpart = partcnt - 1;
  tnet = getnet(tpart);
  tportcnt = tnet->portcnt;

  clear(&xsrc);
  clear(&tsrc);
  clear(&asrc);
  inisrc(&xsrc);
  inisrc(&tsrc);
  inisrc(&asrc);

  dlostop = 0; dhistop = nstop - 1;

  if (portparts[gdep * partcnt + tpart]) { // special case for dep in top
    if (portparts[garr * partcnt + tpart]) return 0;

    dep = tnet->g2pport[gdep];
    error_ge(dep,tportcnt);

    conn = srcxapart(gnet,gdep,garr,nstop,0,&asrc,&tsrc,ref);
    xconn += conn;
  }

  // foreach part with dep as member
  for (part = 0; part < tpart && xconn == 0; part++) {
    if (portparts[gdep * partcnt + part] == 0) continue;

    net = getnet(part);
    portcnt = net->portcnt;
    dep = net->g2pport[gdep];
    error_ge(dep,portcnt);
    tcnt = net->tportcnt;
    info(0,"src xpart %u with %u tports",part,tcnt);

    // foreach port member of top
    for (ti = 0; ti < tcnt && xconn == 0; ti++) {
      dtmid = net->tports[ti];
      if (dtmid >= portcnt) { error(0,"dtmid:%u >= portcnt:%u",dtmid,portcnt); return 0; }

      for (dstop = 0; dstop <= dhistop; dstop++) {
        rv = srclocal(caller,net,part,dep,dtmid,dstop,&xsrc,"xpart dep-top");
        if (rv == 0) continue;

        // dep.part DP has via TD in top
        gdtmid = net->p2gport[dtmid];

        info(0,"part %u gdep %u conn with gmid %u",part,gdep,gdtmid);

        conn = srcxapart(gnet,gdtmid,garr,nstop,dstop,&asrc,&tsrc,ref);
        xconn += conn;
      } // each nstop
    } // each dep.top
  } // each dep.part

  if (xconn == 0) return 0;

  ub4 triplen1 = xsrc.triplen;
  ub4 triplen2 = tsrc.triplen;

#if 0
  error_z(triplen1,0);
  error_z(triplen2,0);
  error_z(xsrc.tripcnt,0);
  error_z(tsrc.tripcnt,0);
#endif

  merge(caller,&xsrc,src,ref);
  merge(caller,&tsrc,src,ref);
  merge(caller,&asrc,src,ref);

  info(0,"src xpart %u with : ",part);
  return xconn;
}

static ub4 srcgeo(struct gnetwork *gnet,ub4 stop,search *src,char *ref)
{
  ub4 gportcnt = gnet->portcnt;
  ub1 *portparts = gnet->portparts;
  ub4 part,partcnt = gnet->partcnt;
  ub4 gdep = src->dep;
  ub4 garr = src->arr;

  ub4 conn;

  if (partcnt == 0) { error(0,"search called without partitions, ref %s",ref); return 0; }
  else if (gportcnt == 0) { error(0,"search without ports, ref %s",ref); return 0; }

  if (partcnt == 1) return srcglocal(gnet,0,gdep,garr,stop,src);

  for (part = 0; part < partcnt; part++) {
    if (portparts[gdep * partcnt + part] == 0 || portparts[garr * partcnt + part] == 0) continue;
    info(0,"dep %u and arr %u share part %u",gdep,garr,part);
    conn = srcglocal(gnet,part,gdep,garr,stop,src);
    if (conn) return conn;
  }
  if (stop == 0) return 0;

  conn = srcxpart(gnet,gdep,garr,stop,src,ref);

  return conn;
}

// rudimentary geographic aka spatial only search
// no time or cost
int searchgeo(search *src,char *ref,ub4 dep,ub4 arr,ub4 nstoplo,ub4 nstophi)
{
  ub4 port;
  ub4 stop,nleg,portno;
  ub4 conn;
  struct gnetwork *net = getgnet();
  ub4 portcnt = net->portcnt;
  struct port *parr,*pdep,*pp,*ports = net->ports;

  if (dep >= portcnt) return error(0,"departure %u not in %u portlist",dep,portcnt);
  if (arr >= portcnt) return error(0,"arrival %u not in %u portlist",arr,portcnt);

  if (nstophi >= Nxstop) { warning(0,"limiting max %u-stop to %u", nstophi,Nxstop); nstophi = Nxstop - 1; }
  if (nstoplo > nstophi) { warning(0,"setting min %u-stop to %u", nstoplo,nstophi); nstoplo = nstophi; }

  info(0,"search geo dep %u arr %u between %u and %u stops, ref %s",dep,arr,nstoplo,nstophi,ref);

  inisrc(src);
  src->dep = dep;
  src->arr = arr;

  pdep = ports + dep;
  parr = ports + arr;

  src->costlim = 0;
  src->lodist = hi32;

  for (stop = 0; stop <= nstophi; stop++) {
    info(CC,"search geo dep %u arr %u in %u-stop %s to %s",dep,arr,stop,pdep->name,parr->name);
    conn = srcgeo(net,stop,src,ref);
    if (conn) break;
  }
  if (src->tripcnt == 0) return info(0,"no route found for %u stops",nstophi);
  nleg = src->lostop + 1;
  info(0,"found %u-%u in %u/%u legs",dep,arr,src->triplen,nleg);
  if (gtriptoports(net,src->trip,src->tripparts,src->triplen,src->tripports)) {
    src->tripcnt = 0;
    return 1;
  }

  for (portno = 0; portno <= nleg; portno++) {
    port = src->tripports[portno];
    pp = ports + port;
    info(0,"port %u %s",port, pp->name);
  }

  return 0;
}

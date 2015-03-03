// netn.c - precompute n-stop connections

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Initialize the network once at startup :

   create a pre-computed connectivity network used in search

   - Build connectivity matrix between any 2 full ports
     derived matrix for each of n intermediate hops

     each matrix contains a list of possible trips from port A to port B
     the list is trimmed on heuristics, such as distance, cost, timing

   - Prepare various metrics used for heuristics

     base matrix for direct (non-stop) hops is prepared in net.c

   one generic function adds 1 stop, combining  connectivity from 0..stop-1
   one dedicated function for the 1-stop net
   one dedicated function for the 2-stop net
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
#include "netn.h"
#include "netev.h"

#undef hdrstop

static int vrbena;

void ininetn(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
  vrbena = (getmsglvl() >= Vrb);
}

#define Distcnt 64
#define Durcnt 64

// create n-stop connectivity matrix and derived info
// uses 1 mid, varying use of underlying nets by stop position
int mknetn(struct network *net,ub4 nstop,ub4 varlimit,ub4 var12limit,bool nilonly)
{
  ub4 part = net->part;
  ub4 portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 whopcnt = net->whopcnt;
  struct port *ports,*pmid,*pdep,*parr;
  char *dname,*aname;
  block *lstblk,*lstblk1,*lstblk2;
  ub4 *portsbyhop;
  ub2 *concnt,*cnts1,*cnts2;
  ub4 *portdst;
  ub4 ofs,ofs1,ofs2,endofs,*conofs,*conofs1,*conofs2;
  ub4 *lst,*newlst,*conlst1,*conlst2,*lst1,*lst11,*lst2,*lst22,*lstv1,*lstv2;
  ub4 *hopdist,*distlims;
  ub4 dep,mid,arr,firstmid,port2,depmid,midarr,deparr,iport1,iport2;
  ub4 iv;
  ub4 cnt,nstop1,n1,n2,n12,altcnt,nleg1,nleg2,v1,v2,leg,leg1,leg2,nleg;
  size_t lstlen,newlstlen;
  ub4 midstop1,midstop2;
  ub4 *lodists;
  ub4 dist1,dist2,distlim,walkdist1,walkdist2,sumwalkdist1,sumwalkdist2;
  ub4 cntlim,cntlimdist,cntlimdur,gen,outcnt;
  ub4 dur,midur,durndx,durcnt,durlim,distcnt,distndx;
  ub4 *hopdur,*durlims;
  ub4 midurs[Durcnt];
  ub4 dists[Distcnt];
  ub4 walklimit = net->walklimit;
  ub4 sumwalklimit = net->sumwalklimit;
  ub4 stat_nocon = 0,stat_partcnt = 0,stat_cntlim = 0,stat_partlimdur = 0,stat_partlimdist = 0;

  // todo
  ub4 portlimit = 9000;
  ub4 lstlimit = 1024 * 1024 * 512;
  ub4 altlimit = min(var12limit * 8,256);

  ub4 dupcode,legport1,legport2;
  ub4 trip1ports[Nleg * 2];
  ub4 trip2ports[Nleg * 2];

  struct eta eta;

  error_z(nstop,0);
  error_ge(nstop,Nstop);
  error_zz(portcnt,hopcnt);

  info(0,"init %u-stop connections for %u port %u hop network",nstop,portcnt,whopcnt);

  port2 = portcnt * portcnt;

  ports = net->ports;

  portsbyhop = net->portsbyhop;

  ub1 *allcnt = net->allcnt;

  concnt = alloc(port2, ub2,0,"net concnt",portcnt);
  lodists = alloc(port2, ub4,0xff,"net lodist",portcnt);

  distlims = alloc(port2, ub4,0,"net distlims",portcnt);
  durlims = alloc(port2, ub4,0,"net durlims",portcnt);

  portdst = alloc(portcnt, ub4,0,"net portdst",portcnt);

  hopdist = net->hopdist;
  hopdur = net->hopdur;

  error_zp(hopdist,0);

  nleg = nstop + 1;
  nstop1 = nstop - 1;

  memset(trip1ports,0xff,sizeof(trip1ports));
  memset(trip2ports,0xff,sizeof(trip1ports));

/* Essentially we do for each (departure,arrival) pair:
   Search for a 'via' port such that trip (departure,via) and (via,arrival) exist
   This is done for a given number of total stops.
   Hence, search for such via with various stops for X and Y in dep-X-via-Y-arr
   Trim list of alternatives based on e.g. distance
   Store result by value. This is memory-intensive but keeps code simple

   In short: foreach dep  foreach arr foreach stopline  foreach a with n1[dep,a] with n2 [a,arr]
*/

/*
  pass 1 : foreach (dep,arr) pair at this #stops:
  bound overall best and worst
  currently, cost is distance only
  create histogram and derive threshold to use as filter in next pass
  estimate size of trip list matrix
  obtain basic stats
*/

  ub8 dupstats[16];
  ub4 cntstats[16];

  lstlen = 0;

  aclear(dupstats);
  aclear(cntstats);

  ub4 dmid,dmidcnt,*dmids = alloc(portcnt * nstop,ub4,0,"net vias",portcnt);
  ub4 *drdeps,*drarrs,*mrdeps,*mrarrs;
  char *mname;
  ub4 dmidcnts[Nstop];
  int fd;
  int dbg,limited = 0;
  ub4 hindx,hidur,hidist;
  char cachefile[256];

  fmtstring(cachefile,"cache/net_%u_part_%u_distlim.in",nstop,part);
  fd = fileopen(cachefile,0);
  if (fd != -1) {
    fileread(fd,distlims,port2 * (ub4)sizeof(*distlims),cachefile);
    fileclose(fd,cachefile);
    info(0,"using dist limit cache file %s",cachefile);
  }

  if (portcnt == 0) return 1;

  // for each departure port
  for (dep = 0; dep < portcnt; dep++) {

    if (progress(&eta,"port %u of %u in pass 1 %u-stop net",dep,portcnt,nstop)) return 1;

    if (dep > portlimit) {
      warning(0,"limiting net by %u ports",portlimit);
      break;
    }

    pdep = ports + dep;
    if (pdep->valid == 0) continue;

    if (lstlen + 2 * port2 > lstlimit / nleg) {
      warncc(limited == 0,0,"limiting net by \ah%u triplets",lstlimit / nleg);
      limited = 1;
      var12limit = varlimit = 2;
    }

    // prepare eligible via's
    dname = pdep->name;
    drdeps = pdep->drids;
    drarrs = pdep->arids;

    for (midstop1 = 0; midstop1 < nstop; midstop1++) {
      cnts1 = net->concnt[midstop1];

      dmid = 0;
      for (mid = 0; mid < portcnt; mid++) {
        if (mid == dep) continue;
        pmid = ports + mid;
        if (pmid->valid == 0) continue;

        depmid = dep * portcnt + mid;

        n1 = cnts1[depmid];
        if (n1 == 0) continue;

        // skip vias only on same route
        mname = pmid->name;
        mrdeps = pmid->drids;
        mrarrs = pmid->arids;

        if (pmid->oneroute) {
          vrb0(0,"skip %u-%u-x on same oneway route %x %s to %s",dep,mid,drdeps[0],dname,mname);
          continue;
        }

        dmids[midstop1 * portcnt + dmid++] = mid;
      }
      dmidcnts[midstop1] = dmid;
    }

    outcnt = 0;

    // for each arrival port
    for (arr = 0; arr < portcnt; arr++) {
      if (arr == dep) continue;

      deparr = dep * portcnt + arr;

      if (nilonly && allcnt[deparr]) { cntstats[9]++; continue; }

      parr = ports + arr;
      if (parr->valid == 0) continue;

      aname = parr->name;

      cnt = cntlim = 0;
      durlim = distlim = hi32;

      // for each #stops between dep-via-arr.
      // e.g. trip dep-a-b-via-c-arr has 2 stops before and 1 after via
      for (midstop1 = 0; midstop1 < nstop; midstop1++) {
        midstop2 = nstop1 - midstop1;

        cnts1 = net->concnt[midstop1];
        cnts2 = net->concnt[midstop2];

        // for each via
        // first obtain distance range
        dmidcnt = dmidcnts[midstop1];
        for (dmid = 0; dmid < dmidcnt; dmid++) {
          mid = dmids[midstop1 * portcnt + dmid];
          if (mid == arr) continue;

          depmid = dep * portcnt + mid;

          n1 = cnts1[depmid];
          error_z(n1,mid);

          midarr = mid * portcnt + arr;
          n2 = cnts2[midarr];
          if (n2 == 0) continue;

          error_ovf(n1,ub2);
          error_ovf(n2,ub2);

          n12 = n1 * n2;
          if (n12 > var12limit) { cntstats[7]++; n12 = var12limit; }
          cnt += n12;
          cntlim = min(cnt,varlimit);
        } // each mid stopover port
      } // each midpoint in stop list dep-a-b-arr

      if (cnt) {  // store info
        lstlen += cntlim;
        concnt[deparr] = (ub2)cntlim;
        outcnt++;
      }

      // limits precomputed from file
//      if (distlims[deparr]) continue;

      // todo: start with limits derived from previous nstop
      // e.g. lodists[da] * 2

      // if too many options, sort on distance.
      if (cnt > varlimit) {
        cntstats[8]++;
        if (nstop == 1) cntlimdist = cntlimdur = cntlim / 2;
        else {
          cntlimdist = cntlim;
          cntlimdur = 0; // not yet, pending support in estdur()
        }
        cntlimdist = min(cntlimdist,Distcnt-1);
        cntlimdur = min(cntlimdur,Durcnt-1);

        // subpass 2: create distance and time top-n lists, derive threshold
        altcnt = 0;
        durcnt = distcnt = 0;
        for (midstop1 = 0; midstop1 < nstop; midstop1++) {

          if (altcnt > altlimit) break;

          midstop2 = nstop1 - midstop1;
          nleg1 = midstop1 + 1;
          nleg2 = midstop2 + 1;

          cnts1 = net->concnt[midstop1];
          cnts2 = net->concnt[midstop2];

          lstblk1 = net->conlst + midstop1;
          lstblk2 = net->conlst + midstop2;

          conlst1 = blkdata(lstblk1,0,ub4);
          conlst2 = blkdata(lstblk2,0,ub4);

          conofs1 = net->conofs[midstop1];
          conofs2 = net->conofs[midstop2];

          for (mid = 0; mid < portcnt; mid++) {
            if (mid == dep || mid == arr) continue;
            depmid = dep * portcnt + mid;

            n1 = cnts1[depmid];
            if (n1 == 0) continue;

            midarr = mid * portcnt + arr;
            n2 = cnts2[midarr];
            if (n2 == 0) continue;
            n12 = n1 * n2;
            altcnt += n12;
            if (altcnt > altlimit) break;

            ofs1 = conofs1[depmid];
            ofs2 = conofs2[midarr];
            error_eq(ofs1,hi32);
            error_eq(ofs2,hi32);

            lst1 = conlst1 + ofs1 * nleg1;
            lst2 = conlst2 + ofs2 * nleg2;

            bound(lstblk1,ofs1 * nleg1,ub4);
            bound(lstblk2,ofs2 * nleg2,ub4);

            // each dep-via alternative, except dep-*-arr-*-via
            for (v1 = 0; v1 < n1; v1++) {
              dist1 = walkdist1 = sumwalkdist1 = 0;
              lst11 = lst1 + v1 * nleg1;

              dupcode = 0;
              for (leg1 = 0; leg1 < nleg1; leg1++) {
                leg = lst11[leg1];
                error_ge(leg,whopcnt);
                dist1 += hopdist[leg];
                if (leg >= chopcnt) {
                  walkdist1 += hopdist[leg];
                  sumwalkdist1 += hopdist[leg];
                } else walkdist1 = 0;
                if (nstop > 3) {
                  trip1ports[leg1 * 2] = portsbyhop[leg * 2];
                  trip1ports[leg1 * 2 + 1] = portsbyhop[leg * 2 + 1];
                }
                if (midstop1) dupcode |= (portsbyhop[leg * 2] == arr || portsbyhop[leg * 2 + 1] == arr);
              }

              if (dupcode) continue;
              if (walkdist1 > walklimit || sumwalkdist1 > sumwalklimit) continue;

              if (durlim != hi32) {
                midur = prepestdur(net,lst11,nleg1);
                if ((distlim != hi32 && dist1 > distlim) && midur > durlim) { cntstats[1]++; continue; }
              } else if (distlim != hi32 && dist1 > distlim) { cntstats[1]++; continue; }
              else midur = hi32;
              if (distlim != hi32 && dist1 > distlim * 10) continue;
//              checktrip(net,lst11,nleg1,dep,mid,dist1);

              for (v2 = 0; v2 < n2; v2++) {
                dist2 = dist1;
                lst22 = lst2 + v2 * nleg2;

                dupcode = 0;
                walkdist2 = walkdist1;
                sumwalkdist2 = sumwalkdist1;
                for (leg2 = 0; leg2 < nleg2; leg2++) {
                  leg = lst22[leg2];
//                  error_ge(leg,whopcnt);
                  dist2 += hopdist[leg];
                  if (leg >= chopcnt) {
                    walkdist2 += hopdist[leg];
                    sumwalkdist2 += hopdist[leg];
                  } else walkdist2 = 0;
                  dur = hopdur[leg];
                  if (dur != hi32 && midur != hi32) midur += dur;
//                  else info(Iter,"hop %u %s to %s no dur",leg,dname,aname);
                  if (nstop > 3) {
                    trip2ports[leg2 * 2] = portsbyhop[leg * 2];
                    trip2ports[leg2 * 2 + 1] = portsbyhop[leg * 2 + 1];
                  }
                  if (midstop2) dupcode |= (portsbyhop[leg * 2] == dep || portsbyhop[leg * 2 + 1] == dep);
                }
                if ((distlim != hi32 && dist2 > distlim) && (durlim != hi32 && midur > durlim)) { cntstats[2]++; continue; }
                else if (distlim != hi32 && dist2 > distlim * 15) continue;
                if (walkdist2 > walklimit || sumwalkdist2 > sumwalklimit) continue;

                if (dupcode) continue;

                if (cntlimdur) midur = estdur(net,lst11,nleg1,lst22,nleg2);

//                checktrip(lst22,nleg2,mid,arr,dist2);

                // filter out repeated 'B' visits in dep-*-B-*-via-*-B-*-arr
                if (nstop > 3) {
                  for (legport1 = 0; legport1 < nleg1 * 2; legport1++) {
                    iport1 = trip1ports[legport1];
                    for (legport2 = 1; legport2 < nleg2 * 2; legport2++) {
                      iport2 = trip2ports[legport2];
                      if (iport1 == iport2) { dupcode = 7; break; }
                    }
                    if (dupcode) break;
                  }
                  if (dupcode) {
                    dupstats[2]++;
                    continue;
                  }
                }

                // maintain top-n list, discard actual trip here

                if (distcnt == 0) {
                  dists[0] = dist2;
                  distcnt = 1;
                } else if (distcnt < cntlimdist) {
                  dists[distcnt++] = dist2;
                } else {
                  hidist = hindx = 0;
                  for (distndx = 0; distndx < cntlimdist; distndx++) {
                    if (dists[distndx] > hidist) { hidist = dists[distndx]; hindx = distndx; }
                  }
                  if (dist2 < hidist) dists[hindx] = dist2;
                  distlim = hidist;
                  error_eq(distlim,hi32);
                }

                // idem for time: insertion sort
                if (midur == hi32 || cntlimdur == 0) continue;

                if (durcnt == 0) {
                  midurs[0] = midur;
                  durcnt = 1;
                } else if (durcnt < cntlimdur) {
                  midurs[durcnt++] = midur;
                } else {
                  hidur = hindx = 0;
                  for (durndx = 0; durndx < cntlimdur; durndx++) {
                    if (midurs[durndx] > hidur) { hidur = midurs[durndx]; hindx = durndx; }
                  }
                  if (midur < hidur) midurs[hindx] = midur;
                  durlim = hidur;
                  error_eq(durlim,hi32);
                }

              } // each v2
            } // each v1

          } // each mid

        } // each midpoint

        if (distcnt < cntlimdist) stat_partlimdist++;
        if (cntlimdur && durcnt < cntlimdur) stat_partlimdur++;
        stat_cntlim++;
      } else {   // not cnt limited
        distlim = hi32;
        durlim = hi32;
      }
      distlims[deparr] = distlim;
      durlims[deparr] = durlim;

    } // each arrival port
    portdst[dep] = outcnt;

  } // each departure port

  for (iv = 0; iv < Elemcnt(dupstats); iv++) if (dupstats[iv]) info(0,"dup %u: \ah%lu",iv,dupstats[iv]);
  for (iv = 0; iv < Elemcnt(cntstats); iv++) if (cntstats[iv]) info(0,"cnt %u: \ah%u",iv,cntstats[iv]);

  info(0,"%u-stop pass 1 done, tentative \ah%lu triplets",nstop,lstlen);

  warncc(lstlen == 0 && nilonly == 0,0,"no connections at %u-stop",nstop);
  if (lstlen == 0) return 0;

  ub4 cnt1,newcnt;
  struct range portdr;
  ub4 ivportdst[32];
  mkhist(caller,portdst,portcnt,&portdr,Elemcnt(ivportdst),ivportdst,"outbounds by port",Vrb);

#if 0
  fmtstring(cachefile,"cache/net_%u_part_%u_distlim",nstop,part);
  fd = filecreate(cachefile,0);
  if (fd != -1) {
    filewrite(fd,distlims,port2 * sizeof(*distlims),cachefile);
    fileclose(fd,cachefile);
  }
#endif

  // prepare list matrix and its offsets
  conofs = alloc(port2, ub4,0xff,"net conofs",portcnt);  // = org

  lstblk = net->conlst + nstop;

  lst = mkblock(lstblk,lstlen * nleg,ub4,Init1,"netv %u-stop conlst",nstop);

  cnts1 = net->concnt[nstop1];
  ofs = newcnt = 0;
  for (deparr = 0; deparr < port2; deparr++) {
    cnt = concnt[deparr];
    cnt1 = cnts1[deparr];
    if (cnt) {
      conofs[deparr] = ofs;
      ofs += cnt;
      if (cnt1 == 0) newcnt++;
    }
  }
  error_ne(ofs,lstlen);
  info(0,"\ah%u new connections",newcnt);

  aclear(dupstats);

  // pass 2: fill based on range and thresholds determined above
  // most code comments from pass 1 apply
  ofs = 0;
  for (dep = 0; dep < portcnt; dep++) {

    if (progress(&eta,"port %u of %u in pass 2 %u-stop net",dep,portcnt,nstop)) return 1;

    if (dep > portlimit) continue;

    pdep = ports + dep;

    dname = pdep->name;

    for (midstop1 = 0; midstop1 < nstop; midstop1++) {
      cnts1 = net->concnt[midstop1];

      dmid = 0;
      for (mid = 0; mid < portcnt; mid++) {
        if (mid == dep) continue;
        pmid = ports + mid;
        if (pmid->valid == 0) continue;

        depmid = dep * portcnt + mid;

        n1 = cnts1[depmid];
        if (n1 == 0) continue;

        if (pmid->oneroute) continue;

        dmids[midstop1 * portcnt + dmid++] = mid;
      }
      dmidcnts[midstop1] = dmid;
    }

    for (arr = 0; arr < portcnt; arr++) {
      if (arr == dep) continue;
      deparr = dep * portcnt + arr;

      cnt = concnt[deparr];
      if (cnt == 0) continue;
      gen = concnt[deparr] = 0;

      dbg = (part == 0 && dep == 1068 && arr == 0);

      distlim = distlims[deparr];
      durlim = durlims[deparr];

//      if (distlim != hi32 || durlim != hi32) info(0,"distlim %u durlim %u",distlim,durlim);

      conofs[deparr] = ofs;
      lstv1 = lst + ofs * nleg;
      error_ge(ofs,lstlen);

      endofs = ofs + cnt - 1;

      error_ge(endofs,lstlen);

      infocc(dbg,0,"port %u-%u concnt %u",dep,arr,gen);

      midstop1 = 0;
      while (midstop1 < nstop && gen < cnt) {
        midstop2 = nstop1 - midstop1;
        nleg1 = midstop1 + 1;
        nleg2 = midstop2 + 1;

        cnts1 = net->concnt[midstop1];
        cnts2 = net->concnt[midstop2];

        lstblk1 = net->conlst + midstop1;
        lstblk2 = net->conlst + midstop2;

        conlst1 = blkdata(lstblk1,0,ub4);
        conlst2 = blkdata(lstblk2,0,ub4);

        dmidcnt = dmidcnts[midstop1];
        dmid = 0; firstmid = hi32;
        while (dmid < dmidcnt && gen < cnt) {
          mid = dmids[midstop1 * nstop + dmid++];
          if (mid == arr) continue;

          depmid = dep * portcnt + mid;
          n1 = cnts1[depmid];

          midarr = mid * portcnt + arr;
          n2 = cnts2[midarr];
          if (n2 == 0) continue;

          if (firstmid == hi32) firstmid = mid;

          conofs1 = net->conofs[midstop1];
          conofs2 = net->conofs[midstop2];

          ofs1 = conofs1[depmid];
          ofs2 = conofs2[midarr];
          lst1 = conlst1 + ofs1 * nleg1;
          lst2 = conlst2 + ofs2 * nleg2;

          bound(lstblk1,ofs1 * nleg1,ub4);
          bound(lstblk2,ofs2 * nleg2,ub4);

          v1 = 0;
          while (v1 < n1 && gen < cnt) {
            dist1 = walkdist1 = sumwalkdist1 = 0;
            lst11 = lst1 + v1 * nleg1;

            dupcode = 0;
            for (leg1 = 0; leg1 < nleg1; leg1++) {
              leg = lst11[leg1];
              error_ge(leg,whopcnt);
              dist1 += hopdist[leg];
              if (leg >= chopcnt) {
                walkdist1 += hopdist[leg];
                sumwalkdist1 += hopdist[leg];
              } else walklimit = 0;
              if (nstop > 3) {
                trip1ports[leg1 * 2] = portsbyhop[leg * 2];
                trip1ports[leg1 * 2 + 1] = portsbyhop[leg * 2 + 1];
              }
              if (midstop1) dupcode |= (portsbyhop[leg * 2] == arr || portsbyhop[leg * 2 + 1] == arr);
            }
            if (dupcode) { v1++; continue; }
            if (walkdist1 > walklimit || sumwalkdist1 > sumwalklimit) { v1++; continue; }

            if (durlim != hi32) {
              midur = prepestdur(net,lst11,nleg1);
              if ((distlim != hi32 && dist1 > distlim) && midur > durlim) { v1++; continue; }
            } else if (distlim != hi32 && dist1 > distlim) { v1++; continue; }
            else midur = hi32;
            if (distlim != hi32 && dist1 > distlim * 10) { v1++; continue; }

            if (nstop > 3) {
              error_ne(trip1ports[0],dep);
              error_eq(trip1ports[0],mid);
              error_eq(trip1ports[0],arr);
              error_eq(trip1ports[midstop1 * 2 + 1],dep);
              error_ne(trip1ports[nleg1 * 2 - 1],mid);
              error_eq(trip1ports[nleg1 * 2 - 1],arr);
            }
//            checktrip(net,lst11,nleg1,dep,mid,dist1);

            v2 = 0;
            while (v2 < n2 && gen < cnt) {
              dist2 = dist1;
              lst22 = lst2 + v2 * nleg2;

              dupcode = 0;
              walkdist2 = walkdist1;
              sumwalkdist2 = sumwalkdist1;
              for (leg2 = 0; leg2 < nleg2; leg2++) {
                leg = lst22[leg2];
                error_ge(leg,whopcnt);
                dist2 += hopdist[leg];
                if (leg >= chopcnt) {
                  walkdist2 += hopdist[leg];
                  sumwalkdist2 += hopdist[leg];
                } else walkdist2 = 0;
                dur = hopdur[leg];
                if (dur != hi32 && midur != hi32) midur += dur;
                if (nstop > 3) {
                  trip2ports[leg2 * 2] = portsbyhop[leg * 2];
                  trip2ports[leg2 * 2 + 1] = portsbyhop[leg * 2 + 1];
                }
                if (midstop2) dupcode |= (portsbyhop[leg * 2] == dep || portsbyhop[leg * 2 + 1] == dep);
              }
              if (dupcode) { v2++; continue; }

              if ((distlim != hi32 && dist2 > distlim) && (durlim != hi32 && midur > durlim)) { v2++; continue; }

              if (walkdist2 > walklimit || sumwalkdist2 > sumwalklimit) { v2++; continue; }

              if (nstop > 3) {
                error_ne(trip2ports[0],mid);
                error_eq(trip2ports[0],dep);
                error_eq(trip2ports[0],arr);
                error_ne(trip2ports[nleg2 * 2 - 1],arr);
                error_eq(trip2ports[nleg2 * 2 - 1],dep);
                error_eq(trip2ports[nleg2 * 2 - 1],mid);
              }

              // filter out repeated visits a-B-c-d-B-f
              if (nstop > 3) {
                for (legport1 = 0; legport1 < nleg1 * 2; legport1++) {
                  iport1 = trip1ports[legport1];
                  for (legport2 = 1; legport2 < nleg2 * 2; legport2++) {
                    iport2 = trip2ports[legport2];
                    if (iport1 == iport2) { dupcode = 7; break; }
                  }
                  if (dupcode) break;
                }
                if (dupcode) {
                  v2++;
                  continue;
                }
              }

//              checktrip(lst22,nleg2,mid,arr,dist2);

              if (durlim != hi32) {
                midur = estdur(net,lst11,nleg1,lst22,nleg2);
                if ((distlim != hi32 && dist2 > distlim) && midur > durlim) { v2++; continue; }
              } else if (distlim != hi32 && dist2 > distlim) { v2++; continue; }
              if (distlim != hi32 && dist2 > distlim * 15) { v2++; continue; }

              lodists[deparr] = min(lodists[deparr],dist2);
              allcnt[deparr] = 1;
              gen++;

              for (leg1 = 0; leg1 < nleg1; leg1++) {
                leg = lst11[leg1];
//                error_ge(leg,whopcnt);
                lstv1[leg1] = leg;
              }
//                memcpy(lstv1,lst11,nleg1 * sizeof(ub4));
//                vrb(CC,"dep %u arr %u mid %u \av%u%p base %p n1 %u n2 %u",dep,arr,mid,nleg,lstv1,lst,n1,n2);
//                checktrip(lstv1,nleg1,dep,mid,dist1);
              lstv2 = lstv1 + nleg1;
              for (leg2 = 0; leg2 < nleg2; leg2++) {
                leg = lst22[leg2];
//                error_ge(leg,whopcnt);
                lstv2[leg2] = leg;
              }
//                memcpy(lstv2,lst22,nleg2 * sizeof(ub4));
//                vrb(CC,"dep %u arr %u mid %u \av%u%p base %p n1 %u n2 %u",dep,arr,mid,nleg,lstv1,lst,n1,n2);
//                checktrip3(lstv1,nleg,dep,arr,mid,dist12);

              lstv1 = lstv2 + nleg2;

              v2++;
            }
            v1++;
          }
          mid++;
        } // each mid

        if (gen < cnt) stat_partcnt++;

        // if none found for any dep-Mid-arr, but mid exists, use first one
        if (cnt && gen == 0 && firstmid != hi32) {
          stat_nocon++;
          vrbcc(vrbena,0,"dep %u arr %u no conn for mid %u cnt %u distlim %u",dep,arr,firstmid,cnt,distlim);
          depmid = dep * portcnt + firstmid;
          midarr = firstmid * portcnt + arr;

          error_z(cnts1[depmid],firstmid);
          error_z(cnts2[midarr],firstmid);

          conofs1 = net->conofs[midstop1];
          conofs2 = net->conofs[midstop2];

          ofs1 = conofs1[depmid];
          ofs2 = conofs2[midarr];
          lst1 = conlst1 + ofs1 * nleg1;
          lst2 = conlst2 + ofs2 * nleg2;

          dist2 = 0;
          for (leg1 = 0; leg1 < nleg1; leg1++) {
            leg = lst1[leg1];
            error_ge(leg,whopcnt);
            dist2 += hopdist[leg];
            lstv1[leg1] = leg;
          }
          lstv2 = lstv1 + nleg1;
          for (leg2 = 0; leg2 < nleg2; leg2++) {
            leg = lst2[leg2];
            error_ge(leg,whopcnt);
            dist2 += hopdist[leg];
            lstv2[leg2] = leg;
          }
          lstv1 = lstv2 + nleg2;
          lodists[deparr] = min(lodists[deparr],dist2);
          gen = 1;
          allcnt[deparr] = 1;
        }

        midstop1++;
      } // each mid stopover port

      error_gt(gen,cnt,arr);
      warncc(cnt && gen == 0,Iter,"dep %u arr %u no conn distlim %u durlim %u",dep,arr,distlim,durlim);

      ofs += gen;
      concnt[deparr] = (ub2)gen;
      infocc(dbg,0,"port %u-%u concnt %u",dep,arr,gen);
    } // each arrival port
  } // each departure port
  newlstlen = ofs;
  error_gt(newlstlen,lstlen,nstop);
  info(0,"pass 2 done, \ah%lu from \ah%lu triplets",newlstlen,lstlen);

  afree(distlims,"net distlims");

  if (lstlen - newlstlen > 1024 * 1024 * 64) {
    newlst = trimblock(lstblk,newlstlen * nleg,ub4);
  } else newlst = lst;

  net->lstlen[nstop] = newlstlen;
  for (ofs = 0; ofs < newlstlen * nleg; ofs++) {
    leg = newlst[ofs];
    error_ge(leg,whopcnt);
  }

  for (iv = 0; iv < Elemcnt(dupstats); iv++) if (dupstats[iv]) info(0,"dup %u: \ah%lu",iv,dupstats[iv]);

  info(0,"no conn %u  partcnt %u  cntlim %u %u %u",stat_nocon,stat_partcnt,stat_cntlim,stat_partlimdist,stat_partlimdur);

  struct range conrange;
  ub4 constats[16];

  aclear(constats);
  mkhist2(concnt,port2,&conrange,Elemcnt(constats),constats,"connection",Info);

  net->concnt[nstop] = concnt;
  net->conofs[nstop] = conofs;

  net->lodist[nstop] = lodists;

  net->lstlen[nstop] = lstlen;

  return 0;
}

// create 1-stop connectivity matrix and derived info
// uses 1 mid, varying use of underlying nets by stop position
int mknet1(struct network *net,ub4 varlimit,ub4 var12limit,bool nilonly)
{
  ub4 part = net->part;
  ub4 partcnt = net->partcnt;
  ub4 nstop = 1;
  ub4 portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 whopcnt = net->whopcnt;
  struct port *ports,*pmid,*pdep,*parr;
  char *dname,*aname;
  block *lstblk,*lstblk1;
  ub4 *portsbyhop;
  ub2 *cnts,*cnts1;
  ub4 *portdst;
  ub4 ofs,ofs1,ofs2,endofs,*conofs,*conofs1;
  ub4 *lst,*newlst,*conlst1,*lst1,*lst11,*lst2,*lst22,*lstv1;
  ub4 *hopdist;
  ub4 dep,mid,arr,firstmid,port2,depmid,midarr,deparr;
  ub4 iv;
  ub4 cnt,n1,n2,n12,altcnt,v1,v2,leg,leg1,leg2,nleg;
  size_t lstlen,newlstlen;
  ub4 *lodists;
  ub4 dist1,dist2,distlim,sumwalkdist1,sumwalkdist2,walkdist1,walkdist2;
  ub4 cntlim,cntlimdist,cntlimdur,gen,outcnt;
  ub4 dur,midur,durndx,durcnt,durlim,distcnt,distndx;
  ub4 *hopdur;
  ub4 midurs[Durcnt];
  ub4 dists[Distcnt];
  ub4 walklimit = net->walklimit;
  ub4 sumwalklimit = net->sumwalklimit;
  ub4 stat_nocon = 0,stat_partcnt = 0,stat_cntlim = 0,stat_partlimdur = 0,stat_partlimdist = 0;
  ub4 stat_altlim = 0,stat_oneroute = 0;

  // todo
  ub4 portlimit = 50000;
  ub4 lstlimit = 1024 * 1024 * 512;
  ub4 altlimit = min(var12limit * 8,256);

  ub4 trip1ports[Nleg * 2];
  ub4 trip2ports[Nleg * 2];

  struct eta eta;

  error_zz(portcnt,hopcnt);

  info(0,"init 1-stop connections for %u port \ah%u hop network",portcnt,whopcnt);

  info(0,"limits: var %u var12 %u alt %u port %u lst \ah%u",varlimit,var12limit,altlimit,portlimit,lstlimit);

  port2 = portcnt * portcnt;

  ports = net->ports;

  portsbyhop = net->portsbyhop;

  ub1 *allcnt = net->allcnt;

  cnts = alloc(port2, ub2,0,"net concnt",portcnt);
  if (partcnt > 1) lodists = alloc(port2, ub4,0xff,"net lodist",portcnt);
  else lodists = NULL;

  ub4 *distlims = alloc(port2, ub4,0,"net distlims",portcnt);
  ub2 *durlims = alloc(port2, ub2,0,"net durlims",portcnt);

  portdst = alloc(portcnt, ub4,0,"net portdst",portcnt);

  hopdist = net->hopdist;
  hopdur = net->hopdur;

  error_zp(hopdist,0);

  nleg = 2;

  memset(trip1ports,0xff,sizeof(trip1ports));
  memset(trip2ports,0xff,sizeof(trip1ports));

/* Essentially we do for each (departure,arrival) pair:
   Search for a 'via' port such that trip (departure,via) and (via,arrival) exist
   Trim list of alternatives based on e.g. distance
   Store result by value. This is memory-intensive but keeps code simple

   In short: foreach dep  foreach arr foreach mid with n1[dep,mid] with n2 [mid,arr]
*/

/*
  pass 1 : foreach (dep,arr) pair at this #stops:
  bound overall best and worst
  currently, cost is distance only
  create histogram and derive threshold to use as filter in next pass
  estimate size of trip list matrix
  obtain basic stats
*/

  ub8 dupstats[16];

  lstlen = 0;

  aclear(dupstats);

  ub4 dmid,dmidcnt,*dmids = alloc(portcnt,ub4,0,"net vias",portcnt);
  ub4 *drdeps,*drarrs,*mrdeps,*mrarrs;
  char *mname;
  int fd;
  int dbg = 0,limited = 0;
  ub4 hindx,hidur,hidist;
  char cachefile[256];

  fmtstring(cachefile,"cache/net_1_part_%u_distlim.in",part);
  fd = fileopen(cachefile,0);
  if (fd != -1) {
    fileread(fd,distlims,port2 * (ub4)sizeof(*distlims),cachefile);
    fileclose(fd,cachefile);
    info(0,"using dist limit cache file %s",cachefile);
  }

  cnts1 = net->concnt[0];
  lstblk1 = net->conlst;
  conlst1 = blkdata(lstblk1,0,ub4);
  conofs1 = net->conofs[0];

  ub4 dmidbins[256];
  ub4 dmidivs = Elemcnt(dmidbins) - 1;;

  aclear(dmidbins);

  // for each departure port
  for (dep = 0; dep < portcnt; dep++) {

    if (progress(&eta,"pass 1 port %u/%u 1-stop net \ah%lu conns",dep,portcnt,lstlen)) return 1;

    if (dep > portlimit) {
      warning(0,"limiting net by %u ports",portlimit);
      break;
    }

    pdep = ports + dep;
    if (pdep->valid == 0) continue;

    if (lstlen + 2 * portcnt > lstlimit / nleg) {
      warncc(limited == 0,0,"limiting net by \ah%u triplets",lstlimit / nleg);
      limited = 1;
      var12limit = varlimit = 2;
    }

    // prepare eligible via's
    dname = pdep->name;
    drdeps = pdep->drids;

    dmid = 0;
    for (mid = 0; mid < portcnt; mid++) {
      if (mid == dep) continue;
      pmid = ports + mid;
      if (pmid->valid == 0) continue;

      depmid = dep * portcnt + mid;

      n1 = cnts1[depmid];
      if (n1 == 0) continue;

      // todo: skip vias only on same route
      mname = pmid->name;
      mrdeps = pmid->drids;
      mrarrs = pmid->arids;

      if (pmid->oneroute) {
        vrb0(0,"skip %u-%u-x on same oneway route %x %s to %s",dep,mid,drdeps[0],dname,mname);
        stat_oneroute++;
        continue;
      }

      dmids[dmid++] = mid;
    }
    dmidcnt = dmid;
    dmidbins[min(dmidcnt,dmidivs)]++;

    outcnt = 0;

    // for each arrival port
    for (arr = 0; arr < portcnt; arr++) {
      if (arr == dep) continue;

      deparr = dep * portcnt + arr;

      if (nilonly && allcnt[deparr]) continue;

      parr = ports + arr;
      if (parr->valid == 0) continue;

      aname = parr->name;

      cnt = cntlim = 0;
      durlim = distlim = hi32;

      // for each via
      // first obtain distance range
      for (dmid = 0; dmid < dmidcnt; dmid++) {
        mid = dmids[dmid];
        if (mid == arr) continue;

        depmid = dep * portcnt + mid;

        n1 = cnts1[depmid];
        error_z(n1,mid);

        midarr = mid * portcnt + arr;
        n2 = cnts1[midarr];
        if (n2 == 0) continue;

        error_ovf(n1,ub2);
        error_ovf(n2,ub2);

        n12 = n1 * n2;
        if (n12 > var12limit) n12 = var12limit;
        cnt += n12;
        cntlim = min(cnt,varlimit);
      } // each mid stopover port

      if (cnt) {  // store info
        lstlen += cntlim;
        cnts[deparr] = (ub2)cntlim;
        outcnt++;
      }

      // limits precomputed from file
//      if (distlims[deparr]) continue;

      // todo: start with limits derived from previous nstop
      // e.g. lodists[da] * 2

      // if too many options, sort on distance.
      if (cnt > varlimit) {
//        info(0,"cnt %u varlimit %u cntlim %u",cnt,varlimit,cntlim);
        cntlimdist = max(cntlim / 4,1);
        cntlimdur = max(cntlim * 3 / 4,1);
        cntlimdist = min(cntlimdist,Distcnt-1);
        cntlimdur = min(cntlimdur,Durcnt-1);

        error_z(cntlimdist,cntlim);
        error_z(cntlimdur,cntlim);

        nsethi(dists,Distcnt);
        nsethi(midurs,Durcnt);

        // subpass 2: create distance and time top-n lists, derive threshold
        altcnt = 0;
        durcnt = distcnt = 0;

        if (altcnt > altlimit) { stat_altlim++; break; }

        for (mid = 0; mid < portcnt; mid++) {
          if (mid == dep || mid == arr) continue;
          depmid = dep * portcnt + mid;

          n1 = cnts1[depmid];
          if (n1 == 0) continue;

          midarr = mid * portcnt + arr;
          n2 = cnts1[midarr];
          if (n2 == 0) continue;
          n12 = n1 * n2;
          altcnt += n12;
          if (altcnt > altlimit) break;

          ofs1 = conofs1[depmid];
          ofs2 = conofs1[midarr];
          error_eq(ofs1,hi32);
          error_eq(ofs2,hi32);

          lst1 = conlst1 + ofs1;
          lst2 = conlst1 + ofs2;

          bound(lstblk1,ofs1,ub4);
          bound(lstblk1,ofs2,ub4);

          // each dep-via alternative
          for (v1 = 0; v1 < n1; v1++) {
            sumwalkdist1 = walkdist1 = 0;
            lst11 = lst1 + v1;

            leg1 = lst11[0];
            error_ge(leg1,whopcnt);
            dist1 = hopdist[leg1];
            if (leg1 >= chopcnt) sumwalkdist1 = walkdist1 = dist1;

            if (durlim != hi32) {
              midur = prepestdur(net,lst11,1);
              if ((distlim != hi32 && dist1 > distlim) && midur > durlim) continue;
            } else if (distlim != hi32 && dist1 > distlim) continue;
            else midur = hi32;
            if (distlim != hi32 && dist1 > distlim * 10) continue;

            for (v2 = 0; v2 < n2; v2++) {
              dist2 = dist1;
              lst22 = lst2 + v2;

              sumwalkdist2 = sumwalkdist1;
              walkdist2 = walkdist1;
              leg2 = lst22[0];
              dist2 += hopdist[leg2];
              if (leg2 >= chopcnt) {
                walkdist2 += hopdist[leg2];
                sumwalkdist2 += hopdist[leg2];
              } else walkdist2 = 0;
              dur = hopdur[leg2];
              if (dur != hi32 && midur != hi32) midur += dur;
              if ((distlim != hi32 && dist2 > distlim) && (durlim != hi32 && midur > durlim)) continue;
              else if (distlim != hi32 && dist2 > distlim * 15) continue;
              if (walkdist2 > walklimit || sumwalkdist2 > sumwalklimit) continue;

              // maintain top-n list, discard actual trip here

              if (distcnt == 0) {
                dists[0] = dist2;
                distcnt = 1;
              } else if (distcnt < cntlimdist) {
                dists[distcnt++] = dist2;
              } else {
                hidist = hindx = 0;
                for (distndx = 0; distndx < cntlimdist; distndx++) {
                  if (dists[distndx] > hidist) { hidist = dists[distndx]; hindx = distndx; }
                }
                if (dist2 < hidist) dists[hindx] = dist2;
                distlim = hidist;
                error_eq(distlim,hi32);
              }

              // idem for time: insertion sort
              midur = estdur_2(net,leg1,leg2);
              if (midur == hi32) continue;

              if (durcnt == 0) {
                midurs[0] = midur;
                durcnt = 1;
              } else if (durcnt < cntlimdur) {
                midurs[durcnt++] = midur;
              } else {
                hidur = hindx = 0;
                for (durndx = 0; durndx < cntlimdur; durndx++) {
                  if (midurs[durndx] > hidur) { hidur = midurs[durndx]; hindx = durndx; }
                }
                if (midur < hidur) midurs[hindx] = midur;
                durlim = hidur;
                error_eq(durlim,hi32);
              }

            } // each v2
          } // each v1

        } // each mid

        if (distcnt < cntlimdist) stat_partlimdist++;
        if (durcnt < cntlimdur) stat_partlimdur++;
        stat_cntlim++;
        warncc(durlim != hi32 && durlim > 65534,0,"durlim %u exceeds 64k",durlim);
      } else {   // not cnt limited
        distlim = hi32;
        durlim = hi32;
      }
      distlims[deparr] = distlim;
      durlims[deparr] = (ub2)durlim;

    } // each arrival port
    portdst[dep] = outcnt;

  } // each departure port

  for (iv = 0; iv < Elemcnt(dupstats); iv++) if (dupstats[iv]) info(0,"dup %u: \ah%lu",iv,dupstats[iv]);

  info(0,"1-stop pass 1 done, tentative \ah%lu triplets",lstlen);

  info(0,"cntlim %u partlim %u %u altlim %u oneroute %u",stat_cntlim,stat_partlimdist,stat_partlimdur,stat_altlim,stat_oneroute);

  for (iv = 0; iv <= dmidivs; iv++) if (dmidbins[iv] > 64) info(0,"dmids %u: \ah%u",iv,dmidbins[iv]);

  warncc(lstlen == 0 && nilonly == 0,0,"no connections at 1-stop for %u ports",portcnt);
  if (lstlen == 0) return 0;

  ub4 cnt1,newcnt;
  struct range portdr;
  ub4 ivportdst[32];
  mkhist(caller,portdst,portcnt,&portdr,Elemcnt(ivportdst),ivportdst,"outbounds by port",Vrb);

  ub4 genstats[64];
  ub4 cntstats[64];
  ub4 geniv = 63;

  aclear(cntstats);
  aclear(genstats);

#if 0
  fmtstring(cachefile,"cache/net_1_part_%u_distlim",part);
  fd = filecreate(cachefile,0);
  if (fd != -1) {
    filewrite(fd,distlims,port2 * sizeof(*distlims),cachefile);
    fileclose(fd,cachefile);
  }
#endif

  // prepare list matrix and its offsets
  conofs = alloc(port2, ub4,0xff,"net conofs",portcnt);  // = org

  lstblk = net->conlst + nstop;

  lst = mkblock(lstblk,lstlen * nleg,ub4,Noinit,"netv %u-stop conlst",nstop);

  ofs = newcnt = 0;

  if (portcnt < 10000) {
    for (deparr = 0; deparr < port2; deparr++) {
      cnt = cnts[deparr];
      cnt1 = cnts1[deparr];
      if (cnt) {
        conofs[deparr] = ofs;
        ofs += cnt;
        if (cnt1 == 0) newcnt++;
      }
    }
    info(0,"\ah%u new connections",newcnt);
  } else {
    for (deparr = 0; deparr < port2; deparr++) {
      cnt = cnts[deparr];
      if (cnt) {
        conofs[deparr] = ofs;
        ofs += cnt;
      }
    }
  }
  error_ne(ofs,lstlen);

  aclear(dupstats);

  // pass 2: fill based on range and thresholds determined above
  // most code comments from pass 1 apply
  ofs = 0;
  for (dep = 0; dep < portcnt; dep++) {

    if (progress(&eta,"pass 2 port %u/%u 1-stop net \ah%u conns",dep,portcnt,ofs)) return 1;

    if (dep > portlimit) continue;

    pdep = ports + dep;

    dname = pdep->name;
    drdeps = pdep->drids;
    drarrs = pdep->arids;

    dmid = 0;
    for (mid = 0; mid < portcnt; mid++) {
      if (mid == dep) continue;
      pmid = ports + mid;
      if (pmid->valid == 0) continue;

      depmid = dep * portcnt + mid;

      n1 = cnts1[depmid];
      if (n1 == 0) continue;

      if (pmid->oneroute) continue;

      dmids[dmid++] = mid;
    }
    dmidcnt = dmid;

    for (arr = 0; arr < portcnt; arr++) {
      if (arr == dep) continue;
      deparr = dep * portcnt + arr;

      cnt = cnts[deparr];
      if (cnt == 0) continue;
      gen = cnts[deparr] = 0;

      distlim = distlims[deparr];
      durlim = durlims[deparr];
      if (durlim == hi16) durlim = hi32;

//      if (distlim != hi32 || durlim != hi32) info(0,"distlim %u durlim %u",distlim,durlim);

      conofs[deparr] = ofs;
      lstv1 = lst + ofs * nleg;
      error_ge(ofs,lstlen);

      endofs = ofs + cnt - 1;

      error_ge(endofs,lstlen);

      infocc(dbg,0,"port %u-%u concnt %u",dep,arr,gen);

      firstmid = hi32;
      for (dmid = 0; dmid < dmidcnt; dmid++) {
        mid = dmids[dmid];

        if (mid == arr) continue;
        depmid = dep * portcnt + mid;

        midarr = mid * portcnt + arr;
        n2 = cnts1[midarr];
        if (n2 == 0) continue;

        n1 = cnts1[depmid];
        error_z(n1,mid);

        if (firstmid == hi32) firstmid = mid;

        ofs1 = conofs1[depmid];
        ofs2 = conofs1[midarr];
        lst1 = conlst1 + ofs1;
        lst2 = conlst1 + ofs2;

        bound(lstblk1,ofs1,ub4);
        bound(lstblk1,ofs2,ub4);

        for (v1 = 0; v1 < n1; v1++) {
          dist1 = walkdist1 = sumwalkdist1 = 0;
          lst11 = lst1 + v1;

          leg1 = lst11[0];
          error_ge(leg1,whopcnt);
          error_ne(portsbyhop[leg1 * 2],dep);
          error_ne(portsbyhop[leg1 * 2 + 1],mid);
          dist1 += hopdist[leg1];
          if (leg1 >= chopcnt) {
            walkdist1 = hopdist[leg1];
            sumwalkdist1 = hopdist[leg1];
          } else walkdist1 = 0;
          if (durlim != hi32) {
            midur = prepestdur(net,lst11,1);
            if ((distlim != hi32 && dist1 > distlim) && midur > durlim) continue;
          } else if (distlim != hi32 && dist1 > distlim) continue;
            else midur = hi32;
          if (distlim != hi32 && dist1 > distlim * 10) continue;

          for (v2 = 0; v2 < n2; v2++) {
            dist2 = dist1;
            lst22 = lst2 + v2;

            sumwalkdist2 = sumwalkdist1;
            walkdist2 = walkdist1;
            leg2 = lst22[0];
            error_ge(leg2,whopcnt);
            error_ne(portsbyhop[leg2 * 2],mid);
            error_ne(portsbyhop[leg2 * 2 + 1],arr);

            dist2 += hopdist[leg2];
            if (leg2 >= chopcnt) {
              walkdist2 += hopdist[leg2];
              sumwalkdist2 += hopdist[leg2];
            } else walkdist2 = 0;
            dur = hopdur[leg2];
            if (dur != hi32 && midur != hi32) midur += dur;

            if ((distlim != hi32 && dist2 > distlim) && (durlim != hi32 && midur > durlim)) continue;
            if (walkdist2 > walklimit || sumwalkdist2 > sumwalklimit) continue;

            if (durlim != hi32) {
              midur = estdur(net,lst11,1,lst22,1);
              if ((distlim != hi32 && dist2 > distlim) && midur > durlim) continue;
            } else if (distlim != hi32 && dist2 > distlim) continue;
            if (distlim != hi32 && dist2 > distlim * 15) continue;

            if (lodists) lodists[deparr] = min(lodists[deparr],dist2);
            allcnt[deparr] = 1;
            gen++;

            lstv1[0] = leg1;
            lstv1[1] = leg2;

//            checktrip3(net,lstv1,2,dep,arr,mid,dist2);

            lstv1 += 2;

            if (gen >= cnt) break;
          } // each v2
          if (gen >= cnt) break;
        } // each v1

        if (gen >= cnt) break;
      } // each mid

      if (gen < cnt) {
        stat_partcnt++;
        infocc(dmid < dmidcnt,0,"dmid %u of %u",dmid,dmidcnt);
      }

      genstats[min(gen,geniv)]++;
      cntstats[min(cnt,geniv)]++;

      // if none found for any dep-Mid-arr, but mid exists, use first one
      if (cnt && gen == 0 && firstmid != hi32) {
        stat_nocon++;
        vrbcc(vrbena,0,"dep %u arr %u no conn for mid %u cnt %u distlim %u",dep,arr,firstmid,cnt,distlim);
        depmid = dep * portcnt + firstmid;
        midarr = firstmid * portcnt + arr;

        error_z(cnts1[depmid],firstmid);
        error_z(cnts1[midarr],firstmid);

        ofs1 = conofs1[depmid];
        ofs2 = conofs1[midarr];
        lst1 = conlst1 + ofs1;
        lst2 = conlst1 + ofs2;

        leg1 = lst1[0];
        error_ge(leg1,whopcnt);
        dist2 = hopdist[leg1];
        lstv1[0] = leg1;
        leg2 = lst2[0];
        error_ge(leg2,whopcnt);
        dist2 += hopdist[leg2];
        lstv1[1] = leg2;
        lstv1 += 2;
        if (lodists) lodists[deparr] = min(lodists[deparr],dist2);
        gen = 1;
        allcnt[deparr] = 1;
      }

      error_gt(gen,cnt,arr);
      warncc(cnt && gen == 0,Iter,"dep %u arr %u no conn distlim %u durlim %u",dep,arr,distlim,durlim);

      ofs += gen;
      cnts[deparr] = (ub2)gen;
      infocc(dbg,0,"port %u-%u concnt %u",dep,arr,gen);
    } // each arrival port
  } // each departure port
  newlstlen = ofs;
  error_gt(newlstlen,lstlen,0);
  info(0,"pass 2 done, \ah%lu from \ah%lu triplets",newlstlen,lstlen);

  afree(distlims,"net distlims");
  afree(durlims,"net durlims");

  if (lstlen - newlstlen > 1024 * 1024 * 64) {
    newlst = trimblock(lstblk,newlstlen * nleg,ub4);
  } else newlst = lst;

  net->lstlen[1] = newlstlen;

  for (ofs = 0; ofs < newlstlen * nleg; ofs++) {
    leg = newlst[ofs];
    error_ge(leg,whopcnt);
  }

  for (iv = 0; iv < Elemcnt(dupstats); iv++) if (dupstats[iv]) info(0,"dup %u: \ah%lu",iv,dupstats[iv]);

  info(0,"no conn %u  partcnt %u  cntlim %u partlim1 %u %u",stat_nocon,stat_partcnt,stat_cntlim,stat_partlimdist,stat_partlimdur);

  for (iv = 0; iv <= geniv; iv++) infocc(cntstats[iv],0,"%u: gen \ah%u cnt \ah%u",iv,genstats[iv],cntstats[iv]);

  // verify all triplets
  if (portcnt < 10000) {
    for (dep = 0; dep < portcnt; dep++) {
      for (arr = 0; arr < portcnt; arr++) {
        if (dep == arr) continue;

        deparr = dep * portcnt + arr;
        n1 = cnts[deparr];
        if (n1 == 0) continue;
        ofs = conofs[deparr];
        lstv1 = newlst + ofs * nleg;
        for (v1 = 0; v1 < n1; v1++) {
          checktrip(net,lstv1,nleg,dep,arr,hi32);
          lstv1 += nleg;
        }
      }
    }
  }

  struct range conrange;
  ub4 constats[16];

  aclear(constats);

  if (portcnt < 10000) mkhist2(cnts,port2,&conrange,Elemcnt(constats),constats,"connection",Info);

  net->concnt[1] = cnts;
  net->conofs[1] = conofs;

  net->lodist[1] = lodists; // only for partitioned

  net->lstlen[1] = lstlen;

  return 0;
} // end mknet1

// create 2-stop connectivity matrix and derived info
// uses 2 mids, varying use of underlying nets for each
int mknet2(struct network *net,ub4 varlimit,ub4 var12limit,bool nilonly)
{
  ub4 part = net->part;
  ub4 portcnt = net->portcnt;
  ub4 hopcnt = net->hopcnt;
  ub4 chopcnt = net->chopcnt;
  ub4 whopcnt = net->whopcnt;
  ub4 nstop = 2;
  struct port *ports,*pmid,*pdep,*parr;
  char *dname,*aname;
  block *lstblk,*lstblk1;
  ub4 *portsbyhop;
  ub4 *hoprids;
  ub2 *cnts,*cnts1;
  ub4 *portdst;
  ub4 ofs,ofs1,ofs2,ofs3,endofs,*conofs,*conofs1;
  ub4 *lst,*newlst,*conlst1,*lst1,*lst11,*lst2,*lst22,*lst3,*lst33,*lstv1;
  ub4 *hopdist,*distlims;
  ub4 dep,mid1,mid2,arr,port2,depmid1,mid12,mid2arr,deparr;
  ub4 iv;
  ub4 cnt,n1,n2,n3,n123,altcnt,v1,v2,v3,leg,leg1,leg2,leg3,nleg;
  size_t lstlen,newlstlen;
  ub4 *lodists;
  ub4 dist1,dist2,dist3,distlim,walkdist1,walkdist2,walkdist3,sumwalkdist1,sumwalkdist2,sumwalkdist3;
  ub4 cntlim,cntlimdist,cntlimdur,gen,outcnt;
  ub4 dur,midur,durndx,durcnt,durlim,distcnt,distndx;
  ub4 *hopdur,*durlims;
  ub4 midurs[Durcnt];
  ub4 dists[Distcnt];
  ub4 walklimit = net->walklimit;
  ub4 sumwalklimit = net->sumwalklimit;
  ub4 stat_nocon = 0,stat_partcnt = 0,stat_cntlim = 0,stat_partlimdur = 0,stat_partlimdist = 0;
  ub4 stat_altlim = 0,stat_oneroute = 0;

  // todo
  ub4 portlimit = 9000;
  ub4 lstlimit = 1024 * 1024 * 512;
  ub4 altlimit = min(var12limit * 4,128);
  ub4 dmidlim = 16;

  ub4 trip1ports[Nleg * 2];
  ub4 trip2ports[Nleg * 2];

  struct eta eta;

  error_zz(portcnt,hopcnt);

  info(0,"init %u-stop connections for %u port %u hop network",nstop,portcnt,whopcnt);

  info(0,"limits: var %u var12 %u alt %u port %u lst \ah%u",varlimit,var12limit,altlimit,portlimit,lstlimit);

  port2 = portcnt * portcnt;

  ports = net->ports;

  portsbyhop = net->portsbyhop;

  ub1 *allcnt = net->allcnt;

  cnts = alloc(port2, ub2,0,"net concnt",portcnt);
  lodists = alloc(port2, ub4,0xff,"net lodist",portcnt);

  distlims = alloc(port2, ub4,0,"net distlims",portcnt);
  durlims = alloc(port2, ub4,0,"net durlims",portcnt);

  portdst = alloc(portcnt, ub4,0,"net portdst",portcnt);

  hopdist = net->hopdist;
  hopdur = net->hopdur;
  hoprids = net->hoprids;

  error_zp(hopdist,0);

  nleg = 3;

  memset(trip1ports,0xff,sizeof(trip1ports));
  memset(trip2ports,0xff,sizeof(trip1ports));

/* Essentially we do for each (departure,arrival) pair:
   Search for 2 'via' ports such that trip (dep,via1), (via1,via2) and (via2,arr) exist
   Trim list of alternatives based on e.g. distance
   Store result by value. This is memory-intensive but keeps code simple

   In short:
   foreach dep
     foreach arr
       foreach mid1 with [dep,mid1]
         foreach mid2 with [mid1,mid2] and [mid2,arr]
           ...

*/ 

/*
  pass 1 : foreach (dep,arr) pair at this #stops:
  bound overall best and worst
  currently, cost is distance only
  create histogram and derive threshold to use as filter in next pass
  estimate size of trip list matrix
  obtain basic stats
*/

  ub8 dupstats[16];
  ub4 cntstats[16];

  lstlen = 0;

  aclear(dupstats);
  aclear(cntstats);

  ub4 dmid,dmidcnt,*dmids = alloc(portcnt,ub4,0,"net vias",portcnt);
  ub4 amid,amidcnt,*amids = alloc(portcnt,ub4,0,"net vias",portcnt);
  int fd;
  int dbg = 0,limited = 0;
  ub4 hindx,hidur,hidist;
  char cachefile[256];

  fmtstring(cachefile,"cache/net_1_part_%u_distlim.in",part);
  fd = fileopen(cachefile,0);
  if (fd != -1) {
    fileread(fd,distlims,port2 * (ub4)sizeof(*distlims),cachefile);
    fileclose(fd,cachefile);
    info(0,"using dist limit cache file %s",cachefile);
  }

  cnts1 = net->concnt[0];
  lstblk1 = net->conlst;
  conlst1 = blkdata(lstblk1,0,ub4);
  conofs1 = net->conofs[0];

  ub4 dmidbins[256];
  ub4 dmidivs = Elemcnt(dmidbins) - 1;;

  aclear(dmidbins);

  // for each departure port
  for (dep = 0; dep < portcnt; dep++) {

    if (progress(&eta,"pass 1 port %u/%u 2-stop net \ah%lu conns",dep,portcnt,lstlen)) return 1;

    if (dep > portlimit) {
      warning(0,"limiting net by %u ports",portlimit);
      break;
    }

    pdep = ports + dep;
    if (pdep->valid == 0) continue;

    if (lstlen + 2 * port2 > lstlimit / nleg) {
      warncc(limited == 0,0,"limiting net by \ah%u triplets",lstlimit / nleg);
      limited = 1;
      var12limit = varlimit = 2;
    }

    dname = pdep->name;

    // prepare eligible via's
    dmid = 0;
    for (mid1 = 0; mid1 < portcnt; mid1++) {
      if (mid1 == dep) continue;
      pmid = ports + mid1;
      if (pmid->valid == 0) continue;

      depmid1 = dep * portcnt + mid1;

      n1 = cnts1[depmid1];
      if (n1 == 0) continue;

      if (pmid->oneroute) {
        stat_oneroute++;
        continue;
      }
      dmids[dmid++] = mid1;
      if (dmid >= dmidlim) break;
    }
    dmidcnt = dmid;
    dmidbins[min(dmidcnt,dmidivs)]++;

    outcnt = 0;

    // for each arrival port
    for (arr = 0; arr < portcnt; arr++) {
      if (arr == dep) continue;

      deparr = dep * portcnt + arr;

      if (nilonly && allcnt[deparr]) { cntstats[9]++; continue; }

      parr = ports + arr;
      if (parr->valid == 0) continue;

      aname = parr->name;

      amid = 0;
      for (mid2 = 0; mid2 < portcnt; mid2++) {
        if (mid2 == dep || mid2 == arr) continue;
        pmid = ports + mid2;
        if (pmid->valid == 0) continue;

        mid2arr = mid2 * portcnt + arr;

        n2 = cnts1[mid2arr];
        if (n2 == 0) continue;

        if (pmid->oneroute) continue;

        amids[amid++] = mid2;
      }
      amidcnt = amid;

      cnt = cntlim = 0;
      durlim = distlim = hi32;

      // for each via1
      // first obtain distance range
      for (dmid = 0; dmid < dmidcnt; dmid++) {
        mid1 = dmids[dmid];
        if (mid1 == arr) continue;

        depmid1 = dep * portcnt + mid1;

        n1 = cnts1[depmid1];
        error_z(n1,mid1);

        for (amid = 0; amid < amidcnt; amid++) {
          mid2 = amids[amid];
          if (mid2 == mid1) continue;

          mid12 = mid1 * portcnt + mid2;
          n2 = cnts1[mid12];
          if (n2 == 0) continue;

          mid2arr = mid2 * portcnt + arr;
          n3 = cnts1[mid2arr];
          if (n3 == 0) continue;

          error_ovf(n1,ub2);
          error_ovf(n2,ub2);

          n123 = n1 * n2 * n3;
          if (n123 > var12limit) { cntstats[7]++; n123 = var12limit; }
          cnt += n123;
          cntlim = min(cnt,varlimit);
        } // each mid2
      } // each mid1

      if (cnt) {  // store info
        lstlen += cntlim;
        cnts[deparr] = (ub2)cntlim;
        outcnt++;
      }

      // limits precomputed from file
//      if (distlims[deparr]) continue;

      // todo: start with limits derived from previous nstop
      // e.g. lodists[da] * 2

      // if too many options, sort on distance.
      if (cnt > varlimit) {
        cntstats[8]++;
        cntlimdist = cntlimdur = cntlim / 2;
        cntlimdist = min(cntlimdist,Distcnt-1);
        cntlimdur = min(cntlimdur,Durcnt-1);

        error_z(cntlimdist,cntlim);
        error_z(cntlimdur,cntlim);

        nsethi(dists,Distcnt);
        nsethi(midurs,Durcnt);

        // subpass 2: create distance and time top-n lists, derive threshold
        altcnt = 0;
        durcnt = distcnt = 0;

        for (dmid = 0; dmid < dmidcnt; dmid++) {
          mid1 = dmids[dmid];
          if (mid1 == arr) continue;

          depmid1 = dep * portcnt + mid1;
          n1 = cnts1[depmid1];

          for (amid = 0; amid < amidcnt; amid++) {
            mid2 = amids[amid];
            if (mid2 == mid1) continue;

            mid12 = mid1 * portcnt + mid2;
            n2 = cnts1[mid12];
            if (n2 == 0) continue;

            mid2arr = mid2 * portcnt + arr;
            n3 = cnts1[mid2arr];
            if (n3 == 0) continue;

            n123 = n1 * n2 * n3;
            altcnt += n123;
            if (altcnt > altlimit) { stat_altlim++; break; }

            ofs1 = conofs1[depmid1];
            ofs2 = conofs1[mid12];
            ofs3 = conofs1[mid2arr];
            error_eq(ofs1,hi32);
            error_eq(ofs2,hi32);
            error_eq(ofs3,hi32);

            lst1 = conlst1 + ofs1;
            lst2 = conlst1 + ofs2;
            lst3 = conlst1 + ofs3;

            bound(lstblk1,ofs1,ub4);
            bound(lstblk1,ofs2,ub4);
            bound(lstblk1,ofs3,ub4);

            // each dep-via alternative, except dep-*-arr-*-via
            for (v1 = 0; v1 < n1; v1++) {
              dist1 = sumwalkdist1 = 0;
              lst11 = lst1 + v1;

              leg1 = lst11[0];
              error_ge(leg1,whopcnt);
              dist1 += hopdist[leg1];
              if (leg1 >= chopcnt) sumwalkdist1 = hopdist[leg1];

              if (durlim != hi32) {
                midur = prepestdur(net,lst11,1);
                if ((distlim != hi32 && dist1 > distlim) && midur > durlim) { cntstats[1]++; continue; }
              } else if (distlim != hi32 && dist1 > distlim) { cntstats[1]++; continue; }
              else midur = hi32;
              if (distlim != hi32 && dist1 > distlim * 10) continue;

              for (v2 = 0; v2 < n2; v2++) {
                dist2 = dist1;
                lst22 = lst2 + v2;

                walkdist2 = sumwalkdist2 = sumwalkdist1;
                leg2 = lst22[0];

                if (hoprids[leg1] == hoprids[leg2] && hoprids[leg1] != hi32) continue;

                dist2 += hopdist[leg2];
                if (leg2 >= chopcnt) {
                  sumwalkdist2 += hopdist[leg2];
                  walkdist2 += hopdist[leg2];
                  if (walkdist2 > walklimit || sumwalkdist2 > sumwalklimit) continue;
                } else walkdist2 = 0;

                dur = hopdur[leg2];
                if (dur != hi32 && midur != hi32) midur += dur;

                if ((distlim != hi32 && dist2 > distlim) && (durlim != hi32 && midur > durlim)) { cntstats[2]++; continue; }
                else if (distlim != hi32 && dist2 > distlim * 15) continue;

                for (v3 = 0; v3 < n3; v3++) {
                  dist3 = dist2;
                  lst33 = lst3 + v3;

                  walkdist3 = walkdist2;
                  sumwalkdist3 = sumwalkdist2;

                  leg3 = lst33[0];
                  dist3 += hopdist[leg3];
                  if (leg3 >= chopcnt) {
                    walkdist3 += hopdist[leg3];
                    sumwalkdist3 += hopdist[leg3];
                    if (walkdist3 > walklimit || sumwalkdist3 > sumwalklimit) continue;
                  }

                  if (distlim != hi32 && dist3 > distlim * 15) continue;

                  midur = estdur_3(net,leg1,leg2,leg3);
                  if ((distlim != hi32 && dist3 > distlim) && (durlim != hi32 && midur != hi32 && midur > durlim)) { cntstats[2]++; continue; }

                  // maintain top-n list, discard actual trip here

                  if (distcnt == 0) {
                    dists[0] = dist3;
                    distcnt = 1;
                  } else if (distcnt < cntlimdist) {
                    dists[distcnt++] = dist3;
                  } else {
                    hidist = hindx = 0;
                    for (distndx = 0; distndx < cntlimdist; distndx++) {
                      if (dists[distndx] > hidist) { hidist = dists[distndx]; hindx = distndx; }
                    }
                    if (dist3 < hidist) dists[hindx] = dist3;
                    distlim = hidist;
                    error_eq(distlim,hi32);
                  }

                  // idem for time: insertion sort
                  if (midur == hi32) continue;

                  if (durcnt == 0) {
                    midurs[0] = midur;
                    durcnt = 1;
                  } else if (durcnt < cntlimdur) {
                    midurs[durcnt++] = midur;
                  } else {
                    hidur = hindx = 0;
                    for (durndx = 0; durndx < cntlimdur; durndx++) {
                      if (midurs[durndx] > hidur) { hidur = midurs[durndx]; hindx = durndx; }
                    }
                    if (midur < hidur) midurs[hindx] = midur;
                    durlim = hidur;
                    error_eq(durlim,hi32);
                  }

                } // each v3
              } // each v2
            } // each v1

          } // each mid2
          if (altcnt > altlimit) break;

        } // each mid1

        if (distcnt < cntlimdist) stat_partlimdist++;
        if (durcnt < cntlimdur) stat_partlimdur++;
        stat_cntlim++;
      } else {   // not cnt limited
        distlim = hi32;
        durlim = hi32;
      }
      distlims[deparr] = distlim;
      durlims[deparr] = durlim;

    } // each arrival port
    portdst[dep] = outcnt;

  } // each departure port

  for (iv = 0; iv < Elemcnt(dupstats); iv++) if (dupstats[iv]) info(0,"dup %u: \ah%lu",iv,dupstats[iv]);
  for (iv = 0; iv < Elemcnt(cntstats); iv++) if (cntstats[iv]) info(0,"cnt %u: \ah%u",iv,cntstats[iv]);

  info(0,"2-stop pass 1 done, tentative \ah%lu triplets",lstlen);

  info(0,"cntlim %u partlim %u %u altlim %u",stat_cntlim,stat_partlimdist,stat_partlimdur,stat_altlim);

  for (iv = 0; iv <= dmidivs; iv++) if (dmidbins[iv]) info(0,"dmids %u: \ah%u",iv,dmidbins[iv]);

  warncc(lstlen == 0 && nilonly == 0,0,"no connections at 2-stop for %u ports",portcnt);
  if (lstlen == 0) return 0;

  ub4 cnt1,newcnt;
  struct range portdr;
  ub4 ivportdst[32];
  mkhist(caller,portdst,portcnt,&portdr,Elemcnt(ivportdst),ivportdst,"outbounds by port",Vrb);

#if 0
  fmtstring(cachefile,"cache/net_2_part_%u_distlim",part);
  fd = filecreate(cachefile,0);
  if (fd != -1) {
    filewrite(fd,distlims,port2 * sizeof(*distlims),cachefile);
    fileclose(fd,cachefile);
  }
#endif

  // prepare list matrix and its offsets
  conofs = alloc(port2, ub4,0xff,"net conofs",portcnt);  // = org

  lstblk = net->conlst + nstop;
  lst = mkblock(lstblk,lstlen * nleg,ub4,Init1,"netv %u-stop conlst",nstop);

  ofs = newcnt = 0;
  for (deparr = 0; deparr < port2; deparr++) {
    cnt = cnts[deparr];
    cnt1 = cnts1[deparr];
    if (cnt) {
      conofs[deparr] = ofs;
      ofs += cnt;
      if (cnt1 == 0) newcnt++;
    }
  }
  error_ne(ofs,lstlen);
  info(0,"\ah%u new connections",newcnt);

  aclear(dupstats);

  // pass 2: fill based on range and thresholds determined above
  // most code comments from pass 1 apply
  ofs = 0;

  for (dep = 0; dep < portcnt; dep++) {

    if (progress(&eta,"pass 2 port %u/%u 2-stop net \ah%u conns",dep,portcnt,ofs)) return 1;

    if (dep > portlimit) continue;

    dmid = 0;
    for (mid1 = 0; mid1 < portcnt; mid1++) {
      if (mid1 == dep) continue;
      pmid = ports + mid1;
      if (pmid->valid == 0) continue;

      depmid1 = dep * portcnt + mid1;

      n1 = cnts1[depmid1];
      if (n1 == 0) continue;

      if (pmid->oneroute) continue;

      dmids[dmid++] = mid1;
      if (dmid >= dmidlim) break;
    }
    dmidcnt = dmid;

    for (arr = 0; arr < portcnt; arr++) {
      if (arr == dep) continue;
      deparr = dep * portcnt + arr;

      cnt = cnts[deparr];
      if (cnt == 0) continue;
      gen = cnts[deparr] = 0;

      amid = 0;
      for (mid2 = 0; mid2 < portcnt; mid2++) {
        if (mid2 == dep || mid2 == arr) continue;
        pmid = ports + mid2;
        if (pmid->valid == 0) continue;

        mid2arr = mid2 * portcnt + arr;

        n2 = cnts1[mid2arr];
        if (n2 == 0) continue;

        if (pmid->oneroute) continue;

        amids[amid++] = mid2;
      }
      amidcnt = amid;

      distlim = distlims[deparr];
      durlim = durlims[deparr];

//      if (distlim != hi32 || durlim != hi32) info(0,"distlim %u durlim %u",distlim,durlim);

      conofs[deparr] = ofs;
      lstv1 = lst + ofs * nleg;
      error_ge(ofs,lstlen);

      endofs = ofs + cnt - 1;

      error_ge(endofs,lstlen);

      infocc(dbg,0,"port %u-%u concnt %u",dep,arr,gen);

      for (dmid = 0; dmid < dmidcnt; dmid++) {
        mid1 = dmids[dmid];

        if (mid1 == arr) continue;

        depmid1 = dep * portcnt + mid1;
        n1 = cnts1[depmid1];

        for (amid = 0; amid < amidcnt; amid++) {
          mid2 = amids[amid];
          if (mid2 == mid1) continue;

          mid12 = mid1 * portcnt + mid2;
          n2 = cnts1[mid12];
          if (n2 == 0) continue;

          mid2arr = mid2 * portcnt + arr;
          n3 = cnts1[mid2arr];
          if (n3 == 0) continue;

          ofs1 = conofs1[depmid1];
          ofs2 = conofs1[mid12];
          ofs3 = conofs1[mid2arr];
          lst1 = conlst1 + ofs1;
          lst2 = conlst1 + ofs2;
          lst3 = conlst1 + ofs3;

          bound(lstblk1,ofs1,ub4);
          bound(lstblk1,ofs2,ub4);
          bound(lstblk1,ofs3,ub4);

          for (v1 = 0; v1 < n1; v1++) {
            dist1 = walkdist1 = 0;
            lst11 = lst1 + v1;

            leg1 = lst11[0];
            error_ge(leg1,whopcnt);
            dist1 += hopdist[leg1];
            if (leg1 >= chopcnt) walkdist1 = hopdist[leg1];
            if (walkdist1 > walklimit) continue;

            if (durlim != hi32) {
              midur = prepestdur(net,lst11,1);
              if ((distlim != hi32 && dist1 > distlim) && midur > durlim) continue;
            } else if (distlim != hi32 && dist1 > distlim) continue;
            else midur = hi32;
            if (distlim != hi32 && dist1 > distlim * 10) continue;

            for (v2 = 0; v2 < n2; v2++) {
              dist2 = dist1;
              lst22 = lst2 + v2;

              walkdist2 = sumwalkdist2 = walkdist1;
              leg2 = lst22[0];

              if (hoprids[leg1] == hoprids[leg2] && hoprids[leg1] != hi32) continue;

              dist2 += hopdist[leg2];
              if (leg2 >= chopcnt) {
                walkdist2 += hopdist[leg2];
                sumwalkdist2 += hopdist[leg2];
                if (walkdist2 > walklimit || sumwalkdist2 > sumwalklimit) continue;
              } else walkdist2 = 0;

              dur = hopdur[leg2];
              if (dur != hi32 && midur != hi32) midur += dur;

              if ((distlim != hi32 && dist2 > distlim) && (durlim != hi32 && midur > durlim)) continue;

              if (durlim != hi32) {
                midur = estdur_2(net,leg1,leg2);
                if ((distlim != hi32 && dist2 > distlim) && midur > durlim) continue;
              } else if (distlim != hi32 && dist2 > distlim) continue;
              if (distlim != hi32 && dist2 > distlim * 15) continue;

              for (v3 = 0; v3 < n3; v3++) {
                dist3 = dist2;
                lst33 = lst3 + v3;

                walkdist3 = walkdist2;
                sumwalkdist3 = sumwalkdist2;

                leg3 = lst33[0];
                dist3 += hopdist[leg3];
                if (leg3 >= chopcnt) {
                  walkdist3 += hopdist[leg3];
                  sumwalkdist3 += hopdist[leg3];
                  if (walkdist3 > walklimit || sumwalkdist3 > sumwalklimit) continue;
                }

                dur = hopdur[leg3];
                if (dur != hi32 && midur != hi32) midur += dur;

                if ((distlim != hi32 && dist3 > distlim) && (durlim != hi32 && midur > durlim)) continue;

                if (durlim != hi32) {
                  midur = estdur_3(net,leg1,leg2,leg3);
                  if ((distlim != hi32 && dist3 > distlim) && midur > durlim) continue;
                } else if (distlim != hi32 && dist3 > distlim) continue;
                if (distlim != hi32 && dist3 > distlim * 15) continue;

                // candidate passed, store by value
                lodists[deparr] = min(lodists[deparr],dist3);
                allcnt[deparr] = 1;
                gen++;

                leg = lst11[0];
                lstv1[0] = lst11[0];
                lstv1[1] = lst22[0];
                lstv1[2] = lst33[0];

                lstv1 += nleg;

                if (gen >= cnt) break;
              } // each v3
              if (gen >= cnt) break;
            } // each v2
            if (gen >= cnt) break;
          } // each v1

          if (gen >= cnt) break;
        } // each mid2

        if (gen >= cnt) break;
      } // each mid1

      if (gen < cnt) stat_partcnt++;

      error_gt(gen,cnt,arr);
      warncc(cnt && gen == 0,Iter,"dep %u arr %u no conn distlim %u durlim %u",dep,arr,distlim,durlim);

      ofs += gen;
      cnts[deparr] = (ub2)gen;
      infocc(dbg,0,"port %u-%u concnt %u",dep,arr,gen);
      error_gt(ofs,lstlen,0);

    } // each arrival port

  } // each departure port

  newlstlen = ofs;
  error_gt(newlstlen,lstlen,0);
  info(0,"pass 2 done, \ah%lu from \ah%lu triplets",newlstlen,lstlen);

  afree(distlims,"net distlims");
  afree(durlims,"net durlims");

  if (lstlen - newlstlen > 1024 * 1024 * 64) {
    newlst = trimblock(lstblk,newlstlen * nleg,ub4);
  } else newlst = lst;

  net->lstlen[nstop] = newlstlen;
  for (ofs = 0; ofs < newlstlen * nleg; ofs++) {
    leg = newlst[ofs];
    error_ge(leg,whopcnt);
  }

  for (iv = 0; iv < Elemcnt(dupstats); iv++) if (dupstats[iv]) info(0,"dup %u: \ah%lu",iv,dupstats[iv]);

  info(0,"no conn %u  partcnt %u  cntlim %u",stat_nocon,stat_partcnt,stat_cntlim);

  // verify all triplets
  for (dep = 0; dep < portcnt; dep++) {
    for (arr = 0; arr < portcnt; arr++) {
      if (dep == arr) continue;

      deparr = dep * portcnt + arr;
      n1 = cnts[deparr];
      if (n1 == 0) continue;
      ofs = conofs[deparr];
      lstv1 = newlst + ofs * nleg;
      for (v1 = 0; v1 < n1; v1++) {
        checktrip(net,lstv1,nleg,dep,arr,hi32);
        lstv1 += nleg;
      }
    }
  }

  struct range conrange;
  ub4 constats[16];

  aclear(constats);
  mkhist2(cnts,port2,&conrange,Elemcnt(constats),constats,"connection",Info);

  net->concnt[nstop] = cnts;
  net->conofs[nstop] = conofs;

  net->lodist[nstop] = lodists;

  net->lstlen[nstop] = lstlen;

  return 0;
} // end mknet2

// net.h - main + derived network defines

/* structures and definitions for the main and derived network components
 * search-related defines are in src.h
 */

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/*
 a net consists of ports (aka stops) connected by hops (aka links,connections)
 routes are auxiliary

 connectivity is stored in 2D port by port arrays for each of n stops aka transfers

 for larger #ports, a set of measures are taken to reduce the size

 - merge related ports that can trivially be interpolated
 - partition by set of routes

  1m^2 = 1t -> (1k + 1)^2 * (1k + 1) + (?^2) ~ 1g

 */

#ifndef Netbase_inc
  enum txkind { Unknown,Air,Rail,Bus,Walk,Kindcnt };
#endif

// max number of partitions
#define Npart 8192

#define Nlocal 4

#define Nleg (Nstop+1)
#define Nxleg (Nxstop+1)

#define Chainlen 256

#if (Nstop < 1)
 #error "Nstop must be > 0"
#endif

struct port {
  ub4 magic;
  ub4 id;      // index in net.ports
  ub4 cid;
  ub4 allid;

  char name[128];
  ub4 namelen;

  ub4 gid;   // global port, index in gnet.ports

  ub4 lat,lon;
  double rlat,rlon;

  ub4 utcofs;

  ub4 partcnt;  // #parts member of

  ub4 zid;
  ub4 zlen;
  ub4 zedhop;

  bool tpart; // member of global part

  bool valid;
  bool isagg;
  bool full;
  bool mini;

  ub4 macid;
  ub4 macport;

  ub2 minicnt;
  ub2 macone;
  ub4 miniofs;
  ub4 macsize;

  ub4 macbox[4]; // latlon of mimi members

  ub4 ndep,narr,ngdep,ngarr;   // generic connectivity info

  ub4 nudep,nuarr,nvdep,nvarr; // todo connectivity info
  ub4 nwalkdep,nwalkarr; // todo

  ub4 deps[Nlocal];     // store small net locally
  ub4 arrs[Nlocal];
  ub4 drids[Nlocal];
  ub4 arids[Nlocal];

  ub2 prox0cnt;  // #ports in 0-stop proximity. aka direct neighbours
};

struct chainhop {
  ub4 hop;
  ub4 tdep,tarr;
  ub4 midur;
  ub4 dist;
};

struct chain {
  ub4 rrid,rid;
  ub4 rtid,tid;
  ub4 hopcnt;
  ub4 rhopcnt;
  ub4 hopofs;
  ub4 rhopofs;
  ub8 code;
  ub1 uni;
};

struct timepat {
  ub4 utcofs;
  ub4 ht0,ht1; // min utc overall validity range
  ub4 tdays;
  ub4 gt0;
  ub4 t0,t1;   // relative to above actual event range
  ub4 lodur,hidur,midur,avgdur,duracc;
  ub4 evcnt;
  ub4 genevcnt;
  ub4 sevcnt;
  ub4 evofs;   // offset in net.events
  ub4 dayofs;  // offset in net.evmaps
};

struct hop {
  ub4 magic;
  ub4 gid;

  char name[64];
  ub4 namelen;

  enum txkind kind;

  ub4 dep,arr;    // within part
  ub4 gdep,garr;  // global

  ub4 rrid,rid;

  struct timepat tp;

  ub4 part;

  ub4 dist;
};

struct route {
  ub4 magic;
  ub4 id;

  char name[128];
  ub4 namelen;

  enum txkind kind;

  ub4 rrid;
  ub4 portcnt;
  ub4 hopcnt;
  ub4 hichainlen;

  ub4 chainofs;
  ub4 chaincnt;

  ub4 part;
};

struct carrier {
  ub4 magic;
  ub4 id;
  // todo
};

// a carrier has one or more services on a route
// a service has a set of maps
// each map has entries for each go
// multiple maps are made when needed for duration difference or not fitting in map
struct sidtable {
  ub4 magic;
  ub4 id;
  ub4 sid;

  char name[128];
  ub4 namelen;

  ub4 t0,t1;
};

// holds all for a partition
struct network {
  ub4 part;
  ub4 portcnt,vportcnt;
  ub4 hopcnt,whopcnt,chopcnt;

  ub4 ghopcnt;

  ub4 routecnt;
  ub4 carriercnt;
  ub4 sidcnt;
  ub4 chaincnt;
  ub4 ridcnt,pridcnt;

  ub4 chainhopcnt;

  struct port *ports;
  struct hop *hops;
  struct chain *chains;

  struct route *routes;   // not partitioned
  struct carrier *carriers;  
  struct sidtable *sids;

  struct chainhop *chainhops;  // points to gnet
  ub4 *ridhops;
  ub8 *chainrhops;

  bool istpart;

// timetables: pointer to basenet
  block *eventmem;
  block *evmapmem;
  ub8 *events;       // <time,dur+tid> tuples
  ub2 *evmaps;       // day maps
  ub8 *sevents;      // [samplecnt * chopcnt] dur+time
  ub4 *sevcnts;      // [chopcnt]

// access
  ub4 *g2pport;      // [gportcnt] global to partition port id
  ub4 *p2gport;      // [portcnt]
  ub4 *g2phop;       // [ghopcnt]
  ub4 *p2ghop;       // [hopcnt]

  ub4 *portsbyhop; // [chopcnt * 2] <dep,arr>

  ub4 *dist0;      // [portcnt2] distance
  ub4 *hopdist;    // [chopcnt] idem
  ub4 *hopdur;     // [chopcnt] average duration
//  ub4 *hopcdur;     // [chopcnt] compound constant duration
  ub4 *shopdur;     // [whopcnt] average duration for estdur

  ub4 *choporg;    // [chopcnt * 2] <first,last>

  ub4 *mac2port;   // [nmac < portcnt]

// connection matrices. cached separately ?
  ub2 *con0cnt;    // [port2]  0-stop connections
  ub4 *con0ofs;    // [port2]  offsets in lst

  ub1 *allcnt;     // [port2]  cumulative for n-stop

  ub4 histop;      // highest n-stop connections inited
  ub4 maxstop;     // highest n-stop connections to be inited

  ub4 hirrid;

  ub4 *tid2rtid;   // [chaincnt]
  ub4 hichainlen;

  size_t needconn;    // final required any-stop connectivity
  size_t haveconn[Nstop];
  
// idem for each of n-stop...
  ub2 *concnt[Nstop];  // [port2]
  ub4 *conofs[Nstop];  // [port2]

  block conlst[Nstop];  // [lstlen]
  size_t lstlen[Nstop];

  ub4 *lodist[Nstop];  // [port2] lowest over-route distance

  ub4 *portdst[Nstop];  // [portcnt] #destinations per port

  ub4 disthis[Nstop];   // highest values at end of nstop
  ub4 durhis[Nstop];

  ub1 *conmask;         // [port2] overall connectivity stopmask

// partitions
  ub4 tportcnt;         // number of ports in global part
  ub4 tports[4096];
};
typedef struct network lnet;

struct partition {
  ub4 bbox[9];
};

struct gnetwork {
  ub4 portcnt,zportcnt;
  ub4 hopcnt,chopcnt,zhopcnt;
  ub4 sidcnt;
  ub4 ridcnt;
  ub4 chaincnt;

  ub4 chainhopcnt;

  struct port *ports;
  struct hop *hops;
  struct sidtable *sids;
  struct chain *chains;
  struct route *routes;

  ub4 *port2zport; //  [portcnt]
  ub4 *zport2port; //  [zportcnt]

  ub4 *portsbyhop; // [hopcnt * 2] <dep,arr>

  ub4 *hopdist;    // [chopcnt] distance
  ub4 *hopdur;     // [chopcnt] average / typical duration
  ub4 *hopcdur;     // [chopcnt] compound constant duration
  ub4 *choporg;    // [chopcnt * 2] <first,last>

  ub4 partcnt,tpart;

  struct partition parts[Npart];

  ub4 portcnts[Npart];  // only proper ports
  ub4 hopcnts[Npart];
  ub4 xhopcnts[Npart];

  ub1 *portparts;  // [partcnt * portcnt] port memberships

// local-to-topnet connectivity
  block xpartdmap,xpartamap; // [portcnt * tportcnt] (stopset,part)
  ub2 *xpartdbase,*xpartabase;
  ub4 xpartlen;

// timetables: pointer to basenet
  block *eventmem;
  block *evmapmem;
  ub8 *events;
  ub2 *evmaps;
  ub4 t0,t1;

  struct chainhop *chainhops;
  ub4 *ridhops;
  ub8 *chainrhops;

  ub4 *tid2rtid;   // [chaincnt]

  ub4 histop;     // highest n-stop connections inited

  ub4 hirrid;
  ub4 hichainlen;
};
typedef struct gnetwork gnet;

struct trip {
  ub4 trip[Nxleg * 2]; // part,leg
  ub4 port[Nxleg];
  ub4 t[Nxleg];
  ub4 tid[Nxleg];
  ub4 dur[Nxleg];

  char desc[256];

  ub4 len;
  ub4 cnt;
  ub4 dist;
};

#define triptoports(net,trip,triplen,ports,gports) triptoports_fln(FLN,(net),(trip),(triplen),(ports),(gports))

extern int mknet(ub4 maxstop);
extern struct network *getnet(ub4 part);
extern struct gnetwork *getgnet(void);
extern int triptoports_fln(ub4 fln,struct network *net,ub4 *trip,ub4 triplen,ub4 *ports,ub4 *gports);
extern int gtriptoports(struct gnetwork *net,struct trip *ptrip,char *buf,ub4 buflen,ub4 *ppos,ub4 utcofs);

#define checktrip(net,legs,nleg,dep,arr,dist) checktrip_fln((net),(legs),(nleg),(dep),(arr),(dist),FLN)
#define checktrip3(net,legs,nleg,dep,arr,via,dist) checktrip3_fln((net),(legs),(nleg),(dep),(arr),(via),(dist),FLN)
extern void checktrip_fln(struct network *net,ub4 *legs,ub4 nleg,ub4 dep,ub4 arr,ub4 dist,ub4 fln);
extern void checktrip3_fln(struct network *net,ub4 *legs,ub4 nleg,ub4 dep,ub4 arr,ub4 via,ub4 dist,ub4 fln);

extern ub4 fgeodist(struct port *pdep,struct port *parr);

extern int showconn(struct port *ports,ub4 portcnt,int local);

extern void ininet(void);

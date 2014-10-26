// net.h - main + derived network defines

/* structures and definitions for the main and derived network components
 * search-related defines are in src.h
 */

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

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
  enum txkind { Unknown,Air,Rail,Bus,Walk };
#endif

#define Npart 256

#define Nlocal 4
#define Nxpart 4

struct port {
  ub4 magic;
  ub4 id;      // index in net.ports
  ub4 allid;

  char name[128];  // todo: use below structure instead
  ub4 namelen;
//  struct gname name;

  ub4 gid;   // global port, index in gnet.ports

  ub4 lat,lon;
  double rlat,rlon;

  ub4 utcofs;

  ub4 partcnt;  // #parts member of
  ub4 part;     // 'home' part

  ub2 partnos[Nxpart];  // partnos of membership
  ub4 pmapofs[Nxpart];  // reachability map

  bool isagg;
  bool full;
  bool mini;
  bool onerid;

  ub4 macid;
  ub4 macport;

  ub2 minicnt;
  ub2 macone;
  ub4 miniofs;
  ub4 macsize;

  ub4 macbox[4]; // latlon of mimi members

  ub4 ndep,narr,nudep,nuarr,nvdep,nvarr;   // generic connectivity info
  ub4 nwalkdep,nwalkarr;

  ub4 deps[Nlocal];     // store small net locally
  ub4 arrs[Nlocal];
  ub4 drids[Nlocal];
  ub4 arids[Nlocal];
  ub4 rid;

  ub2 prox0cnt;  // #ports in 0-stop proximity. aka direct neighbours

  ub2 depcnts[Nstop];
};

struct hop {
  ub4 magic;
  ub4 id;
  ub4 gid;

  char name[128];  // todo: use below structure instead
  ub4 namelen;
//  struct gname name;

  enum txkind kind;

  ub4 dep,arr;
  ub4 routeid,rid;

  ub4 part;

  ub4 dist;
};

struct route {
  ub4 magic;
  ub4 id;

  enum txkind kind;

  ub4 routeid;
  ub4 portcnt;
  ub4 hopcnt;

  ub4 dtermport,atermport; // terminus

  // rest todo
};

struct carrier {
  ub4 magic;
  ub4 id;
  struct carrierbase *base;
  // todo
};

// a carrier has one or more services on a route
// a service has a set of maps
// each map has entries for each go
// multiple maps are made when needed for duration difference or not fitting in map
struct timetable {
  ub4 magic;
  ub4 id;
  struct timetablebase *base;

//  todo unrolled aka expanded timetables

};

#define Nleg (Nstop+1)

// holds all for a partition
struct network {
  ub4 part;
  ub4 portcnt,pportcnt,allportcnt;
  ub4 hopcnt,allhopcnt;

  ub4 routecnt;
  ub4 carriercnt;
  ub4 timetablecnt;

  struct port *ports, *allports;
  struct hop *hops,*allhops;

  struct route *routes;
  struct carrier *carriers;  
  struct timetable *timetables;  // [routecnt]

// access
  ub4 *g2pport;      // [gportcnt] global to partition port id
  ub4 *p2gport;      // [portcnt]

  ub4 tthops[Hopcnt];   // index in timetables above

  ub4 fports2ports[Portcnt];

  ub4 *dist0;      // [portcnt2] distance
  ub4 *hopdist;    // [hopcnt] idem
  ub4 *portsbyhop; // [hopcnt * 2] <dep,arr>

  ub4 *mac2port;   // [nmac < portcnt]

// connection matrices. cached separately ?
  ub2 *con0cnt;    // [port2]  0-stop connections
  ub4 *con0ofs;    // [port2]  offsets in lst

  ub4 maxstop;     // highest n-stop connections inited

  ub4 maxrouteid;
  ub4 maxvariants,routevarmask;

  size_t needconn;    // final required any-stop connectivity

// idem for each of n-stop...
  ub2 *concnt[Nstop];  // [port2]
  ub4 *conofs[Nstop];  // [port2]

  block conlst[Nstop];  // [lstlen]
  size_t lstlen[Nstop];

  ub4 *lodist[Nstop];  // [port2] lowest over-route distance
  ub4 *hidist[Nstop];  // [port2] lowest over-route distance

  ub4 *portdst[Nstop];  // [portcnt] #destinations per port

// pools
// ?  datetime *gopool;

// minis
  ub4 minicnt;
  ub4 *minis;

};

struct partition {
  ub4 bbox[9];
};

struct gnetwork {
  ub4 portcnt;
  ub4 hopcnt;

  struct port *ports;
  struct hop *hops;

  ub4 partcnt;

  struct partition parts[Npart];

  ub4 portcnts[Npart];  // only proper ports
  ub4 hopcnts[Npart];

  ub1 *portparts;  // [partcnt * portcnt] port memberships

  block xpartmap;
  ub2 *xpartbase;
  ub4 xpartlen;

  ub4 maxrouteid;
  ub4 maxvariants,routevarmask;
};

extern int mknet(ub4 maxstop);
extern struct network *getnet(ub4 part);
extern struct gnetwork *getgnet(void);
extern int triptoports(struct network *net,ub4 *trip,ub4 triplen,ub4 *ports,ub4 *gports);

#define checktrip(net,legs,nleg,dep,arr,dist) checktrip_fln((net),(legs),(nleg),(dep),(arr),(dist),FLN)
#define checktrip3(net,legs,nleg,dep,arr,via,dist) checktrip3_fln((net),(legs),(nleg),(dep),(arr),(via),(dist),FLN)
extern void checktrip_fln(struct network *net,ub4 *legs,ub4 nleg,ub4 dep,ub4 arr,ub4 dist,ub4 fln);
extern void checktrip3_fln(struct network *net,ub4 *legs,ub4 nleg,ub4 dep,ub4 arr,ub4 via,ub4 dist,ub4 fln);
extern int showconn(struct port *ports,ub4 portcnt);

extern void ininet(void);

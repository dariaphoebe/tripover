// netbase.h - primary network defines

/* structures and definitions for the primary network components :

 - port ( aka stop, station or hub ).
 - hop ( direct connection between 2 ports )
 - route ( sequence of hops, e.g. for rail and bus )
 - timetable ( aka schedule )
 - service. e.g. route for rail+bus, single hop for air

   examples:

   air
     port = Zaventem
     hop = AMS to BRU

  rail
    port = Leuven
    hop = Bruxelles Nord to Mechelen
    route = Amsterdam centraal to Paris Nord

  bus
    port = Bardon
    hop = Elimbah to Beerburrum
    route = 444N

   Derived, secondary data is mostly in net.h

   Search-related defines are in src.h
 */

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define Netbase_inc

enum txkind { Unknown,Air,Rail,Bus,Walk,Kindcnt };

struct portbase {
  ub4 magic;
  ub4 id;
  ub4 cid;     // constant at net changes

  char name[128];  // todo: use below structure instead
  ub4 namelen;
//  struct gname name;

  ub4 ndep,narr;

  ub4 subcnt,subofs;

  bool parentsta;

  bool air;
  bool rail;

  ub4 lat,lon;
  double rlat,rlon;

  ub4 utcofs;

  ub4 size;
};

struct subportbase {
  ub4 id;      // index in net.ports
  ub4 subid;
  ub4 cid;     // constant at net changes

  char name[128];  // todo: use below structure instead
  ub4 namelen;

  ub4 seq;
};

struct hopbase {
  ub4 magic;
  ub4 id;       // index in net.hops
  ub4 cid;
  char name[128];  // todo: use below structure instead
  ub4 namelen;

//  struct gname name;

  ub1 valid;

  enum txkind kind;

  ub4 dep,arr;
  ub4 routeid;

  ub4 timespos;
  ub4 timecnt;
  ub4 evcnt;
  ub4 t0,t1;   // overall date range of timetable : t1 exclusive

  ub4 dist;

  ub4 carriercnt;
  ub4 servicecnt;
};
 
struct routebase {
  ub4 magic;
  ub4 id;       // index in net.routes
  ub4 cid;
  struct gname name;

  ub4 end0,end1;

  ub4 dist;

  ub4 carriercnt;
  ub4 servicecnt;
};

struct carrierbase {
  ub4 magic;
  ub4 id;       // index in net.carriers
  ub4 cid;
  struct gname name;

  ub4 utcofs;
  ub4 hopcnt;
};

// a carrier has one or more services on a route
// a service has a set of maps
// each map has entries for each go
// go contains one or more trips in a repeating pattern
// multiple maps are made when needed for duration difference or not fitting in map
struct sidbase {
  ub4 rsid;
  ub4 sid;
  ub4 cid;

  char name[128];
  ub4 namelen;

  ub4 carrier;
  ub4 route;
  ub4 service;

  ub4 dow;
  ub4 t0,t1;      // tt validity in minutes utc since epoch
  ub4 t0wday;
  ub4 dt;         // granularity of table in minutes

  ub4 refcnt;

  ub2 duration;   // minutes
  ub2 gocnt;      // # entries below

  ub4 gobase;     // datetime gos[gocnt] at net.dtmaps+base :  daysofweek, timesofday map
};

// optional: ~ unrolled timetable
struct fare {
  ub4 carrier;
  ub4 route;
  ub4 service;
  ub4 t0,t1;      // tt validity in minutes utc since epoch

  ub4 depcnt;
  ub4 farecnt;

  ub4 depbase;    // fareinc deps[depcnt * farecnt] at net.faremaps+base
};

// main structure holding everything
struct networkbase {
  ub4 portcnt;
  ub4 subportcnt;
  ub4 hopcnt;
  ub4 sidcnt;
  ub4 routecnt;
  ub4 carriercnt;
  ub4 timetablecnt;

  ub4 timescnt;

  struct portbase *ports;
  struct subportbase *subports;
  struct hopbase *hops;
  struct sidbase *sids;
  struct routebase *routes;
  struct carrierbase *carriers;  
  struct timetablebase *timetables;  // [routecnt]
  ub4 *timesbase;
  ub4 *events;

  struct memblk portmem;
  struct memblk subportmem;
  struct memblk hopmem;
  struct memblk sidmem;
  struct memblk timesmem;
  struct memblk eventmem;

  ub4 latscale,lonscale;
  ub4 latrange[2];
  ub4 lonrange[2];

  ub4 t0,t1;

// workspace
  ub4 *portwrk;   // [portcnt]

// access
  ub4 tthops[Hopcnt];   // index in timetables above
  ub4 farehops[Hopcnt];   // index in fares above

  ub4 *id2ports;         // [maxid]
  ub4 *subid2ports;      // [maxid]
  ub4 *id2hops;          // [maxid]
  ub4 *rsid2sids;
  ub4 maxportid;
  ub4 maxsubportid;
  ub4 maxrouteid;
  ub4 maxsid;

  ub4 maxvariants,routevarmask;

  datetime *gomaps;
  sb2 *faremaps;
};
typedef struct networkbase netbase;

extern netbase *getnetbase(void);
extern int mkrandnet(ub4 portcnt,ub4 hopcnt);
extern int prepbasenet(void);

extern void ininetbase(void);

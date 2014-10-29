// netio.c - networks to and from file

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* functions to read and write tripover internal networks, utility functions to write diagnostics
 */

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "base.h"
#include "cfg.h"
#include "mem.h"
#include "math.h"
#include "util.h"

static ub4 msgfile;
#include "msg.h"

#include "bitfields.h"
#include "netbase.h"

#include "netio.h"

static ub4 pdfscale_lat = 1200;
static ub4 pdfscale_lon = 1200;

static ub4 lat2pdf(ub4 lat,ub4 lolat,ub4 dlat) { return (lat - lolat) * pdfscale_lat / dlat; }
static ub4 lon2pdf(ub4 lon,ub4 lolon,ub4 dlon) { return (lon - lolon) * pdfscale_lon / dlon; }

static const ub4 pdfmaxports = 1000;
static const ub4 pdfmaxhops = 1000;

/* tripover external format: easy manual editing, typical from single gtfs

files contain tab-separated columns

file ports.txt:

# starts comment
letters introduce commands:
  s <latlon scale,ofs>  scale and offset for lat and lon

numbers start a regular 'port' line:
 id name lat lon

numbers default to hexadecimal. prefix capital D for decimal. lon follows lat radix

file hops.txt
  patterned after ports.txt

commands:
# starts comment
tab introduces commands:
 t from to   # overal time validity of following block

any char except tab and newline introduce a regular hop line:
 name id dport aport route.seq dow.hhmm.rep.t0.t1.dur dow.hhmm.rep ...  # mm implies rep=60min
   dport,aport refers to port.id above
   route refers to route.id below
   seq  = hops from route start
   dow = days of week
   hhmm = hour:min
   rep = repetition interval
   t0,t1 is start and end of repetition
   dur = duration


routes.txt  todo
 id name hopcnt
 ...


  tripover internal format todo: meant for larger nets. typ combine sets of external

 gtfs:

  stop  = port
  route = service e.g. 385
  trip  = triplet
  trip-times = timetable
  shapes = net2pdf
  calendar = part of timetable
  calendar-dates = part of timetable

  airline timetables can use gtfs ?

  perl script to process gtfs into tripover external format
  c tool to combine multiple external nets ( e.g sydney + brisbane ) into internal

 */

enum stopopts { Stopopt_child = 1, Stopopt_parent = 2 };

static const char *kindnames[Kindcnt] = { "unknown","air","rail","bus","walk" };

static int memeq(const char *s,const char *q,ub4 n) { return !memcmp(s,q,n); }

// count non-empty and non-comment lines
static ub4 linecnt(const char *name,const char *buf, ub4 len)
{
  ub4 pos = 0,cnt = 0;
  const char nl = '\n';

  while (pos < len) {
    if (buf[pos] == '#') { while (pos < len && buf[pos] != nl) pos++; }
    else if (buf[pos] == nl) pos++;
    else {
      cnt++;
      while (pos < len && buf[pos] != nl) pos++;
    }
  }
  if (len && buf[len-1] != nl) warning(0,"%s has unterminated last line",name);
  info(0,"%s: %u data lines", name,cnt);
  return cnt;
}

static ub1 hexmap[256];

static void mkhexmap(void)
{
  char c;

  memset(hexmap,0xff,256);
  for (c = '0'; c <= '9'; c++) hexmap[(ub4)c] = c - '0';
  for (c = 'a'; c <= 'f'; c++) hexmap[(ub4)c] = c + 10 - 'a';
  for (c = 'A'; c <= 'F'; c++) hexmap[(ub4)c] = c + 10 - 'A';
  hexmap[9] = 0x20;
  hexmap[0x0a] = 0xfe;
  hexmap[0x20] = 0x20;
}

static int __attribute__ ((format (printf,5,6))) parserr(ub4 fln,const char *fname,ub4 linno,ub4 colno,const char *fmt, ...)
{
  va_list ap;
  char buf[1024];
  ub4 pos,len = sizeof(buf);

  pos = fmtstring(buf,"%s.%u.%u: parse error: ",fname,linno,colno);
  if (fmt) {
    va_start(ap,fmt);
    myvsnprintf(buf,pos,len,fmt,ap);
    va_end(ap);
  }
  return errorfln(fln,0,"%s",buf);
}

static int __attribute__ ((format (printf,5,6))) inerr(ub4 fln,const char *fname,ub4 linno,ub4 colno,const char *fmt, ...)
{
  va_list ap;
  char buf[1024];
  ub4 pos,len = sizeof(buf);

  pos = fmtstring(buf,"%s.%u.%u: ",fname,linno,colno);
  if (fmt) {
    va_start(ap,fmt);
    myvsnprintf(buf,pos,len,fmt,ap);
    va_end(ap);
  }
  return errorfln(fln,0,"%s",buf);
}

static int __attribute__ ((format (printf,5,6))) parsewarn(ub4 fln,const char *fname,ub4 linno,ub4 colno,const char *fmt, ...)
{
  va_list ap;
  char buf[1024];
  ub4 pos,len = sizeof(buf);

  pos = fmtstring(buf,"%s.%u.%u: ",fname,linno,colno);
  if (fmt) {
    va_start(ap,fmt);
    myvsnprintf(buf,pos,len,fmt,ap);
    va_end(ap);
  }
  return warningfln(fln,0,"%s",buf);
}

static int showconstats(struct portbase *ports,ub4 portcnt)
{
  struct portbase *pp;
  ub4 nodep,noarr,nodeparr,udeparr1;
  ub4 hidep,hiarr,hidport,hiaport;
  ub4 port,ndep,narr,n;

  ub4 constats[256];
  ub4 depstats[256];
  ub4 arrstats[256];

  ub4 depivs = Elemcnt(depstats) - 1;
  ub4 arrivs = Elemcnt(arrstats) - 1;

  aclear(constats);
  aclear(depstats);
  aclear(arrstats);
  nodep = noarr = nodeparr = udeparr1 = 0;
  hidep = hiarr = hidport = hiaport = 0;

  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    ndep = pp->ndep; narr = pp->narr;
    if (ndep == 0 && narr == 0) { info(0,"port %u %u has no connections - %s",port,pp->id,pp->name); nodeparr++; }
    else if (ndep == 0) { vrb(0,"port %u has no deps - %s",port,pp->name); nodep++; }
    else if (narr == 0) { vrb(0,"port %u has no arrs - %s",port,pp->name); noarr++; }
    if (ndep < 16 && narr < 16) constats[(ndep << 4) | narr]++;
    depstats[min(ndep,depivs)]++;
    arrstats[min(narr,arrivs)]++;
    if (ndep > hidep) { hidep = ndep; hidport = port; }
    if (narr > hiarr) { hiarr = narr; hiaport = port; }
  }
  genmsg(nodeparr ? Info : Vrb,0,"%u of %u ports without connection",nodeparr,portcnt);
  info(0,"%u of %u ports without departures",nodep,portcnt);
  info(0,"%u of %u ports without arrivals",noarr,portcnt);
  for (ndep = 0; ndep < 3; ndep++) {
    for (narr = 0; narr < 3; narr++) {
      n = constats[(ndep << 4) | narr];
      if (n || ndep < 2 || narr < 2) info(0,"%u port\as with %u dep + %u arr", n,ndep,narr);
    }
  }

  for (ndep = 0; ndep <= depivs; ndep++) {
    n = depstats[ndep];
    if (n) info(0,"%u port\as with %u%s dep", n,ndep,ndep == depivs ? "+" : "");
  }
  for (narr = 0; narr <= arrivs; narr++) {
    n = arrstats[narr];
    if (n) info(0,"%u port\as with %u%s arr", n,narr,narr == arrivs ? "+" : "");
  }
  pp = ports + hidport;
  info(0,"port %u deps %u arrs %u %s",hidport,hidep,pp->narr,pp->name);
  pp = ports + hiaport;
  info(0,"port %u deps %u arrs %u %s",hiaport,pp->ndep,hiarr,pp->name);

  return 0;
}

#define Maxval 32

#define Maxname 256

// name id subid lat lon  # comment
static int rdextports(netbase *net,const char *dir)
{
  char fname[512];
  struct myfile mf;
  ub4 rawportcnt,extportcnt,portcnt,port;
  struct portbase *ports,*pp;
  enum states { Out,Num0,Hex1,Dec0,Dec1,Name,Cmd0,Fls};
  enum states state;
  int rv,iscmd,newitem;
  char *buf,*p,*end,c,tab,nl;
  ub4 len,linno,colno,x,namelen,idhi,subidhi,maxid;
  ub4 lat,lon,id,subid,opts;
  ub4 lolon,lolat,hilon,hilat;
  char name[Maxname];
  ub4 namemax = min(Maxname,sizeof(ports->name)) - 2;
  ub4 val,valndx,vals[Maxval];
  ub4 latscale,latscaleline;

  struct extport {
    char name[64];
    ub4 namelen;
    ub4 id, subid;
    ub4 opts;
    bool parent,child;
    ub4 subcnt,subofs,seq;
    ub4 lat,lon;
  };
  struct extport *extports,*ep,*pep;

  aclear(name);
  fmtstring(fname,"%s/ports.txt",dir);

  rv = readfile(&mf,fname,1);
  if (rv) return 1;

  buf = mf.buf;
  len = (ub4)mf.len;
  rawportcnt = linecnt(fname,buf, len);

  if (rawportcnt == 0) return warning(0,"%s is empty",fname);

  extportcnt = 0;
  extports = ep = alloc(rawportcnt,struct extport,0,"ext ports",rawportcnt);

  state = Out;
  namelen = val = valndx = id = idhi = subidhi = maxid = 0;
  iscmd = newitem = 0;
  latscaleline = 0;

  lolat = lolon = hi32;
  hilat = hilon = 0;

  colno = 0;
  linno = 1;
  tab = '\t'; nl = '\n';

  mkhexmap();

  p = buf; end = buf + len;
  while (p < end) {

    c = *p++;

    if (c == nl) { linno++; colno = 1; }
    else colno++;

//    info(0,"state %u c %c",state,c);

    switch(state) {

    case Out:
      valndx = 0;
      iscmd = 0;
      switch (c) {
        case '#': state = Fls; break;
        case '\t': valndx = namelen = 0; aclear(vals); state = Num0; break;
        case '\n': break;
        case ' ': break;
        case '.': iscmd = 1; state = Cmd0; break;
        default: name[0] = c; namelen = 1; state = Name;
      }
      break;

    case Cmd0:
      switch (c) {
        case '#': state = Fls; break;
        case '\t': valndx = namelen = 0; aclear(vals); state = Num0; break;
        case '\n': return parserr(FLN,fname,linno,colno,"unexpected newline");
        case '.': iscmd = 0; name[0] = '.'; namelen = 1; state = Name; break;
        default: name[0] = c; namelen = 1; state = Name;
      }
      break;

    case Name:
      if (c == tab) { valndx = 0; aclear(vals); state = Num0; }
      else if (c == nl) return parserr(FLN,fname,linno,colno,"missing dep arr");
      else if (namelen + 2 < namemax) name[namelen++] = c;
      else if (name[namemax] != '!') {
        parsewarn(FLN,fname,linno,colno,"name exceeds %u",namemax);
        name[namemax] = '!';
      }
      break;

    case Num0:
      switch(c) {
        case '#': if (valndx > 2) newitem = 1; state = Fls; break;
        case '\t': case ' ': break;
        case 'D': state = Dec0; break;
        case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
        case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
          val = hexmap[(ub4)c];
          state = Hex1;
          break;
        default: return parserr(FLN,fname,linno,colno,"expected digit, found '%c'",c);
      }
      break;

    case Hex1:
      if (valndx >= Maxval) return parserr(FLN,fname,linno,colno,"exceeding %u values",valndx);
      x = hexmap[(ub4)c];
      if (x < 16) val = (val << 4) | x;
      else if (x == 0x20) {
        vals[valndx++] = val;
        state = Num0;
      } else if (x == 0xfe) { // newline
        vals[valndx++] = val;
        newitem = 1;
        state = Out;
      } else return parserr(FLN,fname,linno,colno,"expected whitespace after number, found %c",c);
      break;

    case Dec0:
      if (c < '0' || c > '9') return parserr(FLN,fname,linno,colno,"expected decimal digit, found '%c'",c);
      val = c - '0';
      state = Dec1;
      break;

    case Dec1:
      if (c == tab || c == ' ') {
        if (valndx >= Maxval) return parserr(FLN,fname,linno,colno,"exceeding %u values",valndx);
        vals[valndx++] = val;
        state = Num0;
      } else if (c >= '0' && c <= '9') val = (val * 10) + (c - '0');
      else if (c == nl) {
        vals[valndx++] = val;
        newitem = 1;
        state = Out;
      } else return parserr(FLN,fname,linno,colno,"expected decimal digit, found '%c'",c);
      break;

    case Fls:
      if (c == nl) { state = Out; } break;

    }

    if (newitem && iscmd) {
      name[namelen] = 0;
      iscmd = newitem = 0;
      if (namelen == 8 && memeq(name,"latscale",namelen)) {
        if (latscaleline) parsewarn(FLN,fname,linno,colno,"ignore %s previously defined at %u",name,latscaleline);
        else {
          latscaleline = linno;
          latscale = vals[0];
          info(0,"%s : %u",name,latscale);
        }
      } else info(0,"ignore unknown cmd %s %u",name,vals[0]);

    } else if (newitem) {
      newitem = 0;
      error_ge(extportcnt,rawportcnt);
      if (valndx < 4) return parserr(FLN,fname,linno,colno,"missing id,subid,lat,lon args, only %u",valndx);

      id = vals[0];
      subid = vals[1];
      lat = vals[2];
      lon = vals[3];
      if (valndx > 3) opts = vals[4];
      else opts = 0;
      vrb(0,"port %u id %u sub %u opts %x",extportcnt,id,subid,opts);
      if (id > idhi) idhi = id;
      if (subid > subidhi) subidhi = subid;
      ep->id = id;
      ep->subid = subid;
      ep->opts = opts;
      ep->parent = (opts & Stopopt_parent);
      ep->child  = (opts & Stopopt_child);
      if (id != subid && ep->parent) parsewarn(FLN,fname,linno,colno,"parent port %u has parent %u",subid,id);
      if (id == subid && ep->child) parsewarn(FLN,fname,linno,colno,"child port %u has no parent",id);

      if (lat >= 180 * Latscale) { parsewarn(FLN,fname,linno,colno,"port %u lat %u out of range",id,lat); lat = 0; }
      if (lon >= 360 * Lonscale) { parsewarn(FLN,fname,linno,colno,"port %u lon %u out of range",id,lon); lon = 0; }
      ep->lat = lat;
      ep->lon = lon;
      lolat = min(lolat,lat);
      hilat = max(hilat,lat);
      lolon = min(lolon,lon);
      hilon = max(hilon,lon);
      ep->namelen = namelen;
      if (namelen) memcpy(ep->name,name,namelen);
      else { parsewarn(FLN,fname,linno,colno,"port %u has no name",id); }
      extportcnt++;
      ep++;
    }
  }

  ub4 pid,subofs,seq;
  ub4 cnt,extport,subport,subportcnt;
  struct subportbase *subports,*sp;

  // create mappings
  if (idhi > 100 * 1000 * 1000) warning(0,"max port id %u",idhi);
  if (subidhi > 100 * 1000 * 1000) warning(0,"max subport id %u",subidhi);

  ub4 *id2ports = alloc(idhi+1,ub4,0xff,"ext id2port",idhi);
  ub4 *subid2ports = alloc(subidhi+1,ub4,0xff,"ext subid2port",subidhi);

  port = 0;
  for (extport = 0; extport < extportcnt; extport++) {
    ep = extports + extport;
    id = ep->id;
    subid = ep->subid;
    error_gt(id,idhi);
    error_gt(subid,subidhi);

    if (id == subid) {
      if (id2ports[id] != hi32) warning(0,"port ID %u doubly defined as %x", id,id2ports[id]);
      else id2ports[id] = extport;
    }
    if (subid2ports[subid] != hi32) warning(0,"port subID %u doubly defined as %x", subid,subid2ports[subid]);
    else subid2ports[subid] = extport;
    if (id == subid) port++;
  }
  portcnt = port;

  // count members
  subportcnt = 0;
  for (extport = 0; extport < extportcnt; extport++) {
    ep = extports + extport;
    id = ep->id;
    subid = ep->subid;
    if (id == subid) continue;

    pid = id2ports[id];
    error_ge(pid,extportcnt);
    error_eq(pid,extport);
    pep = extports + pid;
    cnt = pep->subcnt;
    pep->subcnt = cnt + 1;
    ep->seq = cnt;
    vrb(0,"port %u has parent %u %s %s",subid,id,ep->name,pep->name);
    subportcnt++;
  }

  info(0,"%u ports, %u subports", portcnt, subportcnt);
  error_gt(portcnt,extportcnt);
  error_ge(subportcnt,extportcnt);

  if (portcnt == 0) return 0;

  ports = mkblock(&net->portmem,portcnt,struct portbase,Init0,"ext ports");
  net->ports = ports;

  if (subportcnt) {
    subports = mkblock(&net->subportmem,subportcnt,struct subportbase,Init0,"ext subports");
    net->subports = subports;
  } else subports = NULL;

  // assign subport ofs
  subofs = 0;
  for (extport = 0; extport < extportcnt; extport++) {
    ep = extports + extport;
    id = ep->id;
    subid = ep->subid;
    if (id != subid) continue;
    cnt = ep->subcnt;
    if (cnt == 0) continue;
    vrb(0,"port %u subcnt %u %s",extport,cnt,ep->name);
    ep->subofs = subofs;
    subofs += cnt;
  }
  error_ne(subofs,subportcnt);

  // fill
  port = subport = 0;
  for (extport = 0; extport < extportcnt; extport++) {
    ep = extports + extport;
    id = ep->id;
    subid = ep->subid;
    opts = ep->opts;
    namelen = ep->namelen;
    lat = ep->lat;
    lon = ep->lon;
    if (id == subid) {
      pp = ports + port;
      pp->id = pp->cid = id;
      pp->parentsta = ep->parent;
      pp->lat = lat;
      pp->lon = lon;
      pp->namelen = namelen;
      if (namelen) memcpy(pp->name,ep->name,namelen);
      cnt = ep->subcnt;
      if (cnt) { // station with members
        subofs = ep->subofs;
        error_ge(subofs,subportcnt);
        pp->subcnt = cnt;
        pp->subofs = subofs;
      } else if (ep->parent) info(0,"parent station %u has no stops %s",id,ep->name);
      port++;
    } else { // sub
      pid = id2ports[id];
      error_ge(pid,extportcnt);
      pep = extports + pid;
      cnt = pep->subcnt;
      subofs = pep->subofs;
      error_ge(subofs,subportcnt);
      seq = ep->seq;
      vrb(0,"seq %u cnt %u",seq,cnt);
      error_z(cnt,pid);
      subport = subofs + seq;
      error_ge(subport,subportcnt);
      sp = subports + subport;
      error_nz(sp->id,subport);
      sp->id = id;
      sp->subid = subid;
      sp->seq = seq;
      sp->namelen = namelen;
      if (namelen) memcpy(sp->name,ep->name,namelen);
    }
  }

  // resequence
  for (id = 0; id <= idhi; id++) id2ports[id] = hi32;
  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    id = pp->id;
    id2ports[id] = port;
    pp->rlat = lat2rad(pp->lat);
    pp->rlon = lon2rad(pp->lon);
  }
  for (subid = 0; subid <= subidhi; subid++) subid2ports[subid] = hi32;
  for (port = 0; port < subportcnt; port++) {
    sp = subports + port;
    subid = sp->subid;
    subid2ports[subid] = port;
  }

  info(0,"read %u ports from %s", portcnt, fname);
  info(0,"bbox lat %u - %u = %u scale %u",lolat,hilat,hilat-lolat,Latscale);
  info(0,"bbox lon %u - %u = %u scale %u",lolon,hilon,hilon-lolon,Lonscale);
  net->portcnt = portcnt;
  net->subportcnt = subportcnt;
  net->id2ports = id2ports;
  net->subid2ports = subid2ports;
  net->maxportid = idhi;
  net->maxsubportid = subidhi;
  net->latrange[0] = lolat;
  net->latrange[1] = hilat;
  net->lonrange[0] = lolon;
  net->lonrange[1] = hilon;
  return 0;
}

// name id dport.id aport.id route.seq (dow.hhmm.rep.t0.t1.dur dow.hhmm.rep)+ 
static int rdexthops(netbase *net,const char *dir)
{
  char fname[512];
  struct myfile mf;
  ub4 rawhopcnt,hopcnt;
  ub4 portcnt,subportcnt;
  ub4 maxportid;
  ub4 variants = 0,variantsline = 0, rtype_walk = 0, rtype_walkline = 0;
  struct hopbase *hops,*hp;
  struct portbase *ports,*pdep,*parr;
  struct subportbase *subports,*sp;
  ub4 *id2ports, *subid2ports;
  enum states { Out,Num0,Hex1,Dec0,Dec1,Name,Cmd0,Fls};
  enum states state;
  int rv,newitem,iscmd;
  char *buf,*p,*end,c,tab,nl;
  ub4 len,linno,colno,x,val,namelen,valndx,id,idhi,maxid,maxrid;
  ub4 depid,arrid,dep,arr,pid,rtype,routeid;
  char name[Maxname];
  ub4 vals[Maxval];
  ub4 namemax = min(Maxname,sizeof(hops->name)) - 1;
  ub4 kinds[Kindcnt];
  enum txkind kind;

  aclear(kinds);

  fmtstring(fname,"%s/hops.txt",dir);

  rv = readfile(&mf,fname,1);
  if (rv) return 1;

  buf = mf.buf;
  len = (ub4)mf.len;
  rawhopcnt = linecnt(fname,buf, len);

  if (rawhopcnt == 0) return warning(0,"%s is empty",fname);
  hopcnt = 0;

  hops = hp = mkblock(&net->hopmem,rawhopcnt,struct hopbase,Init0,"");

  portcnt = net->portcnt;
  subportcnt = net->subportcnt;
  ports = net->ports;
  subports = net->subports;
  id2ports = net->id2ports;
  subid2ports = net->subid2ports;
  maxportid = net->maxportid;

  state = Out;
  namelen = val = valndx = id = idhi = maxid = maxrid = 0;
  newitem = iscmd = 0;

  colno = 0;
  linno = 1;
  tab = '\t'; nl = '\n';

  mkhexmap();

  p = buf; end = buf + len;
  while (p < end) {

    c = *p++;

    if (c == '\n') { linno++; colno = 1; }
    else colno++;

//    info(0,"state %u %c",state,c);

    switch(state) {

    case Out:
      valndx = 0;
      iscmd = 0;
      switch (c) {
        case '#': state = Fls; break;
        case '\t': valndx = namelen = 0; aclear(vals); state = Num0; break;
        case '\n': break;
        case ' ': break;
        case '.': iscmd = 1; state = Cmd0; break;
        default: name[0] = c; namelen = 1; state = Name;
      }
      break;

    case Cmd0:
      switch (c) {
        case '#': state = Fls; break;
        case '\t': valndx = namelen = 0; aclear(vals); state = Num0; break;
        case '\n': return parserr(FLN,fname,linno,colno,"unexpected newline");
        case '.': iscmd = 0; name[0] = '.'; namelen = 1; state = Name; break;
        default: name[0] = c; namelen = 1; state = Name;
      }
      break;

    case Name:
      if (c == tab) { valndx = 0; aclear(vals); state = Num0; }
      else if (c == nl) return parserr(FLN,fname,linno,colno,"missing dep arr");
      else if (namelen + 2 < namemax) name[namelen++] = c;
      else if (name[namemax] != '!') {
        parsewarn(FLN,fname,linno,colno,"name exceeds %u",namemax);
        name[namemax] = '!';
      }
      break;

    case Num0:
      switch(c) {
        case '#': if (valndx > 2) newitem = 1; state = Fls; break;
        case '\t': case ' ': break;
        case 'D': state = Dec0; break;
        case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
        case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
          val = hexmap[(ub4)c];
          state = Hex1;
          break;
        default: return parserr(FLN,fname,linno,colno,"expected digit, found '%c'",c);
      }
      break;

    case Hex1:
      if (valndx >= Maxval) return parserr(FLN,fname,linno,colno,"exceeding %u values",valndx);
      x = hexmap[(ub4)c];
      if (x < 16) val = (val << 4) | x;
      else if (x == 0x20) {
        vals[valndx++] = val;
        state = Num0;
      } else if (x == 0xfe) { // newline
        vals[valndx++] = val;
        newitem = 1;
        state = Out;
      } else return parserr(FLN,fname,linno,colno,"expected whitespace after number, found %c",c);
      break;

    case Dec0:
      if (c < '0' || c > '9') return parserr(FLN,fname,linno,colno,"expected decimal digit, found '%c'",c);
      val = c - '0';
      state = Dec1;
      break;

    case Dec1:
      if (c == tab || c == ' ') {
        if (valndx >= Maxval) return parserr(FLN,fname,linno,colno,"exceeding %u values",valndx);
        vals[valndx++] = val;
        state = Num0;
      } else if (c >= '0' && c <= '9') val = (val * 10) + (c - '0');
      else if (c == nl) {
        vals[valndx++] = val;
        newitem = 1;
        state = Out;
      } else return parserr(FLN,fname,linno,colno,"expected decimal digit, found '%c'",c);
      break;

    case Fls:
      if (c == nl) { state = Out; } break;

    }

    if (newitem && iscmd) {
      name[namelen] = 0;
      iscmd = newitem = 0;
      if (namelen == 8 && memeq(name,"variants",namelen)) {
        if (variantsline) parsewarn(FLN,fname,linno,colno,"ignore %s previously defined at %u",name,variantsline);
        else {
          variantsline = linno;
          variants = vals[0];
          info(0,"%s : %u",name,variants);
        }
      } else if (namelen == 7 && memeq(name,"walk_id",namelen)) {
        if (rtype_walkline) parsewarn(FLN,fname,linno,colno,"ignore %s previously defined at %u",name,rtype_walkline);
        else {
          rtype_walkline = linno;
          rtype_walk = vals[0];
          info(0,"%s : %u",name,rtype_walk);
        }
      } else info(0,"ignore unknown cmd %s %u",name,vals[0]);
    } else if (newitem) {
      newitem = 0;
      error_gt(hopcnt+1,rawhopcnt);
      if (valndx < 4) return parserr(FLN,fname,linno,colno,"missing dep,arr,type args, only %u",valndx);
      id = vals[0];
      depid = vals[1];
      arrid = vals[2];
      rtype = vals[3];
      if (valndx > 4) {
        routeid = vals[4];
        if (routeid != hi32) maxrid = max(maxrid,routeid);
      } else routeid = hi32;

//      info(0,"vals %u %u %u",vals[0],depid,arrid);
      if (depid == arrid) return inerr(FLN,fname,linno,colno,"dep id %u equal to arr id",depid);
      if (depid > maxportid) return inerr(FLN,fname,linno,colno,"dep id %u above highest port id %u",depid,maxportid);
      else if (arrid > maxportid) return inerr(FLN,fname,linno,colno,"arr id %u above highest port id %u",arrid,maxportid);

      dep = id2ports[depid];
      if (dep == hi32) {
        dep = subid2ports[depid];
        if (dep >= subportcnt) return inerr(FLN,fname,linno,colno,"dep %u id %u above highest subport %u",dep,depid,subportcnt);
        sp = subports + dep;
        pid = sp->id;
        dep = id2ports[pid];
      }
      if (dep >= portcnt) return inerr(FLN,fname,linno,colno,"dep %u id %u above highest port %u",dep,depid,portcnt);

      arr = id2ports[arrid];
      if (arr == hi32) {
        arr = subid2ports[arrid];
        if (arr >= subportcnt) return inerr(FLN,fname,linno,colno,"arr %u above highest subport %u",arr,subportcnt);
        sp = subports + arr;
        pid = sp->id;
        arr = id2ports[pid];
      }
      if (arr >= portcnt) return inerr(FLN,fname,linno,colno,"arr %u above highest port %u",arr,portcnt);
      if (dep == arr) {
        info(0,"line %u hop id %u dep %u id %u equal to arr id %u",linno,id,dep,depid,arrid);
        continue;
      }

      pdep = ports + dep;
      parr = ports + arr;
      pdep->ndep++;
      parr->narr++;
      if (pdep->parentsta) vrb(0,"hop %u dport %u %u %s",id,dep,pdep->id,pdep->name);
      if (parr->parentsta) vrb(0,"hop %u aport %u %u %s",id,arr,parr->id,parr->name);

      hp->id  = id;
      hp->dep = dep;
      hp->arr = arr;

      if (rtype_walkline && rtype == rtype_walk) { hp->kind = Walk; kinds[Walk]++; routeid = hi32; }
      else if (rtype == 0 || rtype == 1 || rtype == 2) { hp->kind = Rail; kinds[Rail]++; }
      else if (rtype == 3) { hp->kind = Bus; kinds[Bus]++; }
      else { hp->kind = Unknown; kinds[Unknown]++; }

      hp->routeid = routeid;
      hp->namelen = namelen;
      memcopy(hp->name,name,namelen);
      maxid = max(maxid,id);
      hp++;
      hopcnt++;
    }
  }

  for (kind = 0; kind < Kindcnt; kind++) {
    val = kinds[kind];
    if (val) info(0,"%u %s",val,kindnames[kind]);
  }

  ub4 hop,*id2hops;
  ub4 varmask;

  if (maxid > 100 * 1000 * 1000) warning(0,"max hop id %u",maxid);
  id2hops = alloc(maxid+1,ub4,0xff,"id2hops",maxid);
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    id = hp->id;
    if (id2hops[id] != hi32) warning(0,"hop ID %u doubly defined", id);
    else id2hops[id] = hop;
  }
  net->id2hops = id2hops;

  showconstats(ports,portcnt);

  info(0,"read %u hops from %s", hopcnt, fname);
  net->hopcnt = hopcnt;
  net->hops = hops;
  net->maxrouteid = maxrid;

  // lower routeid bits are variants
  if (variants) varmask = ~(1 << variants);  // add lsb for direction
  else varmask = hi32;
  info(0,"%u possible variants, mask %x", variants,varmask);

  net->maxvariants = variants;
  net->routevarmask = varmask;

  return 0;
}

int readextnet(netbase *net,const char *dir)
{
  int rv;
  ub4 portcnt;

  info(0,"reading base net in tripover external format from dir %s",dir);
  rv = rdextports(net,dir);
  if (rv) return rv;

  portcnt = net->portcnt;
  if (portcnt) net->portwrk = alloc(portcnt,ub4,0,"portwrk",portcnt);

  rv = rdexthops(net,dir);
  info(0,"done reading base net in external format, status %d",rv);
  if (rv) return rv;

  if (globs.writext) rv = net2ext(net);
  if (globs.writpdf) net2pdf(net);
  return rv;
}

static ub4 lat2ext(ub4 lat) { return lat; }
static ub4 lon2ext(ub4 lon) { return lon; }

int net2ext(netbase *net)
{
  int fd;
  char buf[1024];
  ub4 pos,x,y;
  ub4 dec = globs.extdec;

  struct hopbase *hp,*hops = net->hops;
  struct portbase *pp,*ports = net->ports;

  ub4 hop,hopcnt = net->hopcnt;
  ub4 port,portcnt = net->portcnt;
  const char *portsname = "ports.txt";
  const char *hopsname = "hops.txt";

  info(0,"writing %u-ports %u-hops base net to dir '.' in external format",portcnt,hopcnt);

  fd = filecreate(portsname);
  if (fd == -1) return 1;

  pos = fmtstring(buf,"# hops %u ports %u\n# id name lat lon\n",hopcnt,portcnt);
  if (filewrite(fd,buf,pos,portsname)) return 1;

  // temporary: port as number
  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    y = lat2ext(pp->lat);
    x = lon2ext(pp->lon);
    if (dec) pos = fmtstring(buf,"D%u\t%s\tD%u\t%u\n", pp->id,pp->name,y,x);
    else pos = fmtstring(buf,"%x\t%s\t%x\t%x\n", pp->id,pp->name,y,x);
    if (filewrite(fd,buf,pos,portsname)) return 1;
  }
  fileclose(fd,portsname);
  info(0,"wrote %u ports to %s",portcnt,portsname);

  fd = filecreate(hopsname);
  if (fd == -1) return 1;

  pos = fmtstring(buf,"# hops %u ports %u\n# id dep arr\n",hopcnt,portcnt);
  if (filewrite(fd,buf,pos,hopsname)) return 1;

  for(hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    if (dec) pos = fmtstring(buf,"%s\tD%u\tD%u\tD%u\n",hp->name,hop,hp->dep,hp->arr);
    else pos = fmtstring(buf,"%s\t%x\t%x\t%x\n",hp->name,hop,hp->dep,hp->arr);
    if (filewrite(fd,buf,pos,hopsname)) return 1;
  }
  fileclose(fd,hopsname);
  info(0,"wrote %u hops to %s",hopcnt,hopsname);

  info(0,"done writing %u-ports %u-hops base net to dir '.' in external format",portcnt,hopcnt);

  return 0;
}

// write network as page content
// todo overlay planned trip in distinct color
static ub4 addnetpdf(netbase *net, char *buf, ub4 len)
{
  struct portbase *pp,*pdep,*parr,*ports = net->ports;
  struct hopbase *hp,*hops = net->hops;
  ub4 port,portcnt = net->portcnt;
  ub4 hop,hopcnt = net->hopcnt;
  ub4 x,y,x0,y0,x1,y1,arr,dep;
  ub4 lolon,lolat,hilon,hilat,dlon,dlat;
  ub4 pos,n;

  pos = mysnprintf(buf,0,len,"BT /F1 18 Tf 25 25 Td (title) Tj ET\n");

  pos += mysnprintf(buf,pos,len,"BT /F1 16 Tf\n");

  lolon = net->lonrange[0];
  hilon = net->lonrange[1];
  lolat = net->latrange[0];
  hilat = net->latrange[1];
  dlon = max(1,hilon - lolon);
  dlat = max(1,hilat - lolat);

  if (portcnt > pdfmaxports) {
    info(0,"limiting %u ports to %u for pdf",portcnt,pdfmaxports);
    portcnt = pdfmaxports;
  }
  if (hopcnt > pdfmaxhops) {
    info(0,"limiting %u hops to %u for pdf",hopcnt,pdfmaxhops);
   hopcnt = pdfmaxhops;
  }

  // temporary: port as number
  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    y = lat2pdf(pp->lat,lolat,dlat);
    x = lon2pdf(pp->lon,lolon,dlon);
    n = mysnprintf(buf,pos,len,"1 0 0 1 %u %u Tm (%u) Tj ", x,y,port);
    if (n == 0) break;
    pos += n;
  }
  pos += mysnprintf(buf,pos,len,"ET\n");

  pos += mysnprintf(buf,pos,len,"1 w\n");

  // draw direct connection as straight line
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    dep = hp->dep;
    arr = hp->arr;
    pdep = ports + dep;
    parr = ports + arr;
    y0 = lat2pdf(pdep->lat,lolat,dlat);
    x0 = lon2pdf(pdep->lon,lolon,dlon);
    y1 = lat2pdf(parr->lat,lolat,dlat);
    x1 = lon2pdf(parr->lon,lolon,dlon);
    n = mysnprintf(buf,pos,len,"%u %u m %u %u l s\n",x0,y0,x1,y1);
    if (n == 0) break;
    pos += n;
  }
  error_gt(pos,len);
  if (pos == len) warning(0,"pdf output buffer limit \ah%u reached: truncated", len);
  return pos;
}

/* write graphic representation of network:
  ports, hops
  currently single page only
 */
int net2pdf(netbase *net)
{
static char content[64 * 1024 * 1024];
static char pagebuf[64 * 1024];

  int fd;
  ub4 pos,xpos,cpos,xrefpos,obj;
  ub4 plen = sizeof pagebuf;
  ub4 xref[16];
  enum objnos { pdfnil, pdfcat, pdftree,pdfnode,pdfcontent, pdflast };
  const char *name = "net.pdf";

  info(0,"writing %u-ports %u-hops base net to %s in pdf format",net->portcnt,net->hopcnt,name);

  fd = filecreate("net.pdf");
  if (fd == -1) return 1;

  pos = mysnprintf(pagebuf,0,plen,"%%PDF-1.4\n");

  // patterned after simple examples in PDF reference
  xref[pdfcat] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Catalog /Pages %u 0 R >>\nendobj\n",pdfcat,pdftree);

  xref[pdftree] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Pages /Kids [%u 0 R] /Count 1 >>\nendobj\n", pdftree,pdfnode);

  xref[pdfnode] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Page /Parent %u 0 R "
    "/MediaBox [0 0 %u %u] /Contents %u 0 R /Resources"
    " << /Font << /F1 << /Type /Font /Subtype /Type1 /BaseFont /Helvetica >> >> >> >>\n"
    "endobj\n", pdfnode,pdftree,pdfscale_lon, pdfscale_lat,pdfcontent);

  xref[pdfcontent] = pos;

  xpos = pos;
  if (filewrite(fd,pagebuf,pos,name)) return 1;

  cpos = addnetpdf(net,content,sizeof content);

  pos = mysnprintf(pagebuf,0, plen,"%u 0 obj\n << /Length %u >>\nstream\n", pdfcontent, cpos);
  if (filewrite(fd,pagebuf,pos,name)) return 1;
  xpos += pos + cpos;

  if (filewrite(fd,content,cpos,name)) return 1;

  pos = mysnprintf(pagebuf,0,plen,"endstream\nendobj\n");

  xrefpos = xpos + pos;
  pos += mysnprintf(pagebuf,pos,plen,"xref\n0 %u\n",pdflast);

  pos += mysnprintf(pagebuf,pos,plen,"%010u 65535 f \n", 0);
  for (obj = 1; obj <= pdfcontent; obj++) pos += mysnprintf(pagebuf,pos,plen,"%010u 00000 n \n", xref[obj]);

  pos += mysnprintf(pagebuf,pos,plen,"trailer\n << /Size %u /Root %u 0 R >>\n", pdflast, pdfcat);

  pos += mysnprintf(pagebuf,pos,plen,"startxref\n%u\n%s\n", xrefpos,"%%EOF");

  if (filewrite(fd,pagebuf,pos,name)) return 1;

  fileclose(fd,name);

  info(0,"done writing base net to %s in pdf format: \ah%u bytes",name,xrefpos + pos);

  return 0;
}

void ininetio(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

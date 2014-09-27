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
#include "os.h"

static ub4 msgfile;
#include "msg.h"

#include "bitfields.h"
#include "netbase.h"

#include "netio.h"

static ub4 pdfscale_lat = 2000;
static ub4 pdfscale_lon = 2000;

static ub4 lat2pdf(ub4 lat) { return lat * pdfscale_lat / (180 * Latscale); }
static ub4 lon2pdf(ub4 lon) { return lon * pdfscale_lon / (360 * Lonscale); }

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

static int parserr(const char *fname,ub4 linno,const char *fmt, ...)
{
  va_list ap;
  char buf[1024];
  ub4 pos,len = sizeof(buf);

  pos = fmtstring(buf,"%s.%u: parse error: ",fname,linno);
  if (fmt) {
    va_start(ap,fmt);
    pos += myvsnprintf(buf,pos,len,fmt,ap);
    va_end(ap);
  }
  return error(0,"%s",buf);
}

static int parsewarn(const char *fname,ub4 linno,const char *fmt, ...)
{
  va_list ap;
  char buf[1024];
  ub4 pos,len = sizeof(buf);

  pos = fmtstring(buf,"%s.%u: ",fname,linno);
  if (fmt) {
    va_start(ap,fmt);
    pos += myvsnprintf(buf,pos,len,fmt,ap);
    va_end(ap);
  }
  return warning(0,"%s",buf);
}

#define Maxname 256

// s<latlon scale,ofs>
// id name latlon  # comment
static int rdextports(netbase *net,const char *dir)
{
  char fname[512];
  struct myfile mf;
  ub4 rawportcnt,portcnt,port;
  struct portbase *ports,*pp;
  ub4 *id2ports;
  enum states { Out, Idhex1,Iddec0,Iddec1,Name0,Name1,Lat0,Lathex1,Lonhex0,Lonhex1,Latdec0,Latdec1,Londec0,Londec1,Fls};
  enum states state;
  int rv,newport;
  char *buf,*p,*end,c,tab,nl;
  ub4 len,pos,linno,x,namelen,lat,lon,id,idhi,maxid,cc;
  char name[Maxname];
  ub4 namemax = min(Maxname,sizeof(ports->name)) - 1;

  fmtstring(fname,"%s/ports.txt",dir);

  rv = readfile(&mf,fname,1);
  if (rv) return 1;

  buf = mf.buf;
  len = (ub4)mf.len;
  rawportcnt = linecnt(fname,buf, len);

  if (rawportcnt == 0) return warning(0,"%s is empty",fname);
  portcnt = 0;

  ports = pp = mkblock(&net->portmem,rawportcnt,struct portbase,Init0,"");

  state = Out;
  namelen = lat = lon = id = idhi = maxid = 0;
  newport = 0;
  pos = 0;
  linno = 1;
  tab = '\t'; nl = '\n';

  mkhexmap();

  p = buf; end = buf + len;
  while (p < end) {

    c = *p++;
    cc = c;
    if (c == nl) linno++;

//    info(0,"state %u c %c",state,c);

// todo: generalize like hops below
    switch(state) {

    case Out:
      switch (c) {
        case '#': state = Fls; break;
        case 'D': state = Iddec0; break;
        case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
        case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
          id = hexmap[cc];
          state = Idhex1;
          break;
        case 's': info0(0,"scale todo"); state = Fls; break;
        case '\n': break;
        default: warning(0,"unexpected char %c",c); state = Fls;
      }
      break;

    case Idhex1:
      x = hexmap[cc];
      if (x < 16) id = (id << 4) | x;
      else if (x == 0x20) state = Name0;
      else return parserr(fname,linno,"expected whitespace after ID, found %c",c);
      break;

    case Iddec0:
      if (c < '0' || c > '9') return parserr(fname,linno,"expected decimal digit for ID, found %c",c);
      id = c - '0';
      state = Iddec1;
      break;

    case Iddec1:
      if (c == tab) state = Name0;
      else if (c >= '0' && c <= '9') id = (id * 10) + (c - '0');
      else return parserr(fname,linno,"expected decimal digit for ID, found %c",c);
      break;

    case Name0:
      namelen = 0;
      if (c == tab) { warning(0,"%u no name",linno); state = Lat0; }
      else { name[namelen++] = c; state = Name1; }
      break;

    case Name1:
      if (c == tab) state = Lat0;
      else if (namelen + 2 < namemax) name[namelen++] = c;
      else if (name[namemax] != '!') {
        parsewarn(fname,linno,"name exceeds %u",namemax);
        name[namemax] = '!';
      }
      break;

    case Lat0:
      if (c == 'D') state = Latdec0;
      else if (hexmap[cc] < 16) {
        lat = hexmap[cc];
        state = Lathex1;
      } else return parserr(fname,linno,"expected digit for lat, found %c",c);
      break;

    case Lathex1:
      x = hexmap[cc];
      if (x < 16) lat = (lat << 4) | x;
      else if (x == 0x20) state = Lonhex0;
      else return parserr(fname,linno,"expected wdigit for lat, found %c",c);
      break;

    case Lonhex0:
      if (hexmap[cc] < 16) {
        lon = hexmap[cc];
        state = Lonhex1;
      }
      else return parserr(fname,linno,"expected digit for lon, found %c",c);
      break;

    case Lonhex1:
      x = hexmap[cc];
      if (x < 16) lon = (lon << 4) | x;
      else if (x == 0x20) { newport = 1; state = Fls; }
      else if (x == 0xfe) { newport = 1; state = Out; }
      else return parserr(fname,linno,"expected digit for lon, found %c",c);
      break;

    case Latdec0:
      if (c < '0' || c > '9') return parserr(fname,linno,"expected decimal digit for lon, found %c",c);
      lat = c - '0';
      state = Latdec1;
      break;

    case Latdec1:
      if (c == tab) state = Londec0;
      else if (c >= '0' && c <= '9') lat = (lat * 10) + (c - '0');
      else return parserr(fname,linno,"expected decimal digit for lon, found %c",c);
      break;

    case Londec0:
      if (c < '0' || c > '9') return parserr(fname,linno,"expected decimal digit for lon, found %c",c);
      lon = c - '0';
      state = Londec1;
      break;

    case Londec1:
      if (c == tab) { newport = 1; state = Fls; }
      else if (c == nl) { newport = 1; state = Out; }
      else if (c >= '0' && c <= '9') lon = (lon * 10) + (c - '0');
      else return parserr(fname,linno,"expected decimal digit for lon, found %c",c);
      break;

    case Fls:
      if (c == nl) { state = Out; } break;
    }
    if (newport) {
      error_ge(portcnt,rawportcnt);
      newport = 0;
      vrb(0,"port %u id %u",portcnt,id);
      if (id > idhi) idhi = id;
      pp->id = id;
      if (lat >= 180 * Latscale) { parsewarn(fname,linno,"port %u lat %u out of range",id,lat); lat = 0; }
      if (lon >= 360 * Lonscale) { parsewarn(fname,linno,"port %u lon %u out of range",id,lon); lon = 0; }
      pp->lat = lat;
      pp->lon = lon;
      pp->rlat = lat2rad(lat);
      pp->rlon = lon2rad(lon);
//    info(0,"latlon %u %u r %e %e",lat,lon,pp->rlat,pp->rlon);
      pp->namelen = namelen;
      if (namelen) memcpy(pp->name,name,namelen);
      portcnt++;
      pp++;
    }
  }

  if (idhi > 100 * 1000 * 1000) warning(0,"max port id %u",idhi);
  id2ports = alloc(idhi+1,ub4,0xff,"id2port",idhi);
  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    id = pp->id;
    error_gt(id,idhi);
    if (id2ports[id] != hi32) warning(0,"port ID %u doubly defined as %x", id,id2ports[id]);
    else id2ports[id] = port;
  }
  info(0,"read %u ports from %s", portcnt, fname);
  net->portcnt = portcnt;
  net->ports = ports;
  net->id2ports = id2ports;
  net->maxportid = idhi;
  return 0;
}

#define Maxval 64

// name id dport.id aport.id route.seq (dow.hhmm.rep.t0.t1.dur dow.hhmm.rep)+ 
static int rdexthops(netbase *net,const char *dir)
{
  char fname[512];
  struct myfile mf;
  ub4 rawhopcnt,hopcnt,hop;
  ub4 portcnt;
  ub4 maxportid;
  struct hopbase *hops,*hp;
  ub4 *id2hops;
  ub4 *id2ports;
  enum states { Out,Num0,Hex1,Dec0,Dec1,Name,Cmd0,Fls};
  enum states state;
  int rv,newhop;
  char *buf,*p,*end,c,tab,nl;
  ub4 len,pos,linno,x,val,namelen,valndx,id,idhi,maxid;
  ub4 depid,arrid,dep,arr;
  char name[Maxname];
  ub4 vals[Maxval];
  ub4 namemax = min(Maxname,sizeof(hops->name)) - 1;

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
  id2ports = net->id2ports;
  maxportid = net->maxportid;

  state = Out;
  namelen = val = valndx = id = idhi = maxid = 0;
  newhop = 0;

  pos = 0;
  linno = 1;
  tab = '\t'; nl = '\n';

  mkhexmap();

  p = buf; end = buf + len;
  while (p < end) {

    c = *p++;

//    info(0,"state %u %c",state,c);

    switch(state) {

    case Out:
      switch (c) {
        case '#': state = Fls; break;
        case '\t': state = Cmd0; break;
        case '\n': linno++; break;
        case ' ': break;
        default: name[0] = c; namelen = 1; state = Name;
      }
      break;

    case Name:
      if (c == tab) { valndx = 0; aclear(vals); state = Num0; }
      else if (c == nl) return parserr(fname,linno,"missing dep arr");
      else if (namelen + 2 < namemax) name[namelen++] = c;
      else if (name[namemax] != '!') {
        parsewarn(fname,linno,"name exceeds %u",namemax);
        name[namemax] = '!';
      }
      break;

    case Num0:
      switch(c) {
        case 'D': state = Dec0; break;
        case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
        case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
          val = hexmap[(ub4)c];
          state = Hex1;
          break;
      }
      break;
    case Hex1:
      if (valndx >= Maxval) return parserr(fname,linno,"exceeding %u values",valndx);
      x = hexmap[(ub4)c];
      if (x < 16) val = (val << 4) | x;
      else if (x == 0x20) {
        vals[valndx++] = val;
        state = Num0;
      } else if (x == 0xfe) { // newline
        vals[valndx++] = val;
        newhop = 1;
        linno++;
        state = Out;
      } else return parserr(fname,linno,"expected whitespace after ID, found %c",c);
      break;

    case Dec0:
      if (c < '0' || c > '9') return parserr(fname,linno,"expected decimal digit for ID, found %c",c);
      val = c - '0';
      state = Dec1;
      break;

    case Dec1:
      if (c == tab) {
        if (valndx >= Maxval) return parserr(fname,linno,"exceeding %u values",valndx);
        vals[valndx++] = val;
        state = Num0;
      } else if (c >= '0' && c <= '9') val = (val * 10) + (c - '0');
      else if (c == nl) {
        vals[valndx++] = val;
        newhop = 1;
        linno++;
        state = Out;
      } else return parserr(fname,linno,"expected decimal digit for ID, found %c",c);
      break;

    case Cmd0:
//      if (c == 'x') { // todo handle commands here
      state = Out;
      break;

    case Fls:
      if (c == nl) { linno++; state = Out; } break;

    }

    if (newhop) {
      newhop = 0;
      error_gt(hopcnt+1,rawhopcnt);
      if (valndx < 3) return parserr(fname,linno,"missing dep,arr args, only %u",valndx);
      id = vals[0];
      depid = vals[1];
      arrid = vals[2];
//    info(0,"vals %u %u %u",vals[0],depid,arrid);
      error_eq(depid,arrid);
      if (depid > maxportid) return parserr(fname,linno,"dep id %u above highest port id %u",depid,maxportid);
      else if (arrid > maxportid) return parserr(fname,linno,"arr id %u above highest port id %u",arrid,maxportid);
      dep = id2ports[depid];
      arr = id2ports[arrid];
      if (dep >= portcnt) return parserr(fname,linno,"dep %u above highest port %u",dep,portcnt);
      else if (arr >= portcnt) return parserr(fname,linno,"arr %u above highest port %u",arr,portcnt);
      hp->id  = id;
      hp->dep = dep;
      hp->arr = arr;
      hp->namelen = namelen;
      if (namelen) memcpy(hp->name,name,namelen);
      hp++;
      hopcnt++;
    }
  }

#if 0
// later
  if (maxid > 100 * 1000 * 1000) warning(0,"max port id %u",maxid);
  id2hops = alloc(maxid+1,ub4,0,"id2port",maxid);
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    id = hp->id;
    if (id2hops[id] != hi32) warning(0,"hop ID %u doubly defined", id);
    else id2hops[id] = hop;
  }
  net->id2hops = id2hops;
#endif

  info(0,"read %u hops from %s", hopcnt, fname);
  net->hopcnt = hopcnt;
  net->hops = hops;

  return 0;
}

int readextnet(netbase *net,const char *dir)
{
  int rv;

  rv = rdextports(net,dir);
  if (rv) return rv;
  rv = rdexthops(net,dir);
  if (rv) return rv;
  if (globs.writext) rv = net2ext(net);
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

//  dec = 1;

  struct hopbase *hp,*hops = net->hops;
  struct portbase *pp,*ports = net->ports;

  ub4 hop,hopcnt = net->hopcnt;
  ub4 port,portcnt = net->portcnt;

  info(0,"write %u port %u hops basenet to ext",portcnt,hopcnt);

  fd = oscreate("ports.txt");
  if (fd == -1) return 1;

  pos = fmtstring(buf,"# hops %u ports %u\n# id name lat lon\n",hopcnt,portcnt);
  oswrite(fd,buf,pos);

  // temporary: port as number
  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    y = lat2ext(pp->lat);
    x = lon2ext(pp->lon);
    if (dec) pos = fmtstring(buf,"D%u\t%s\tD%u\t%u\n", pp->id,pp->name,y,x);
    else pos = fmtstring(buf,"%x\t%s\t%x\t%x\n", pp->id,pp->name,y,x);
    oswrite(fd,buf,pos);
  }
  osclose(fd);

  fd = oscreate("hops.txt");
  if (fd == -1) return 1;

  pos = fmtstring(buf,"# hops %u ports %u\n# id dep arr\n",hopcnt,portcnt);
  oswrite(fd,buf,pos);

  for(hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    if (dec) pos = fmtstring(buf,"%s\tD%u\tD%u\tD%u\n",hp->name,hop,hp->dep,hp->arr);
    else pos = fmtstring(buf,"%s\t%x\t%x\t%x\n",hp->name,hop,hp->dep,hp->arr);
    oswrite(fd,buf,pos);
  }
  osclose(fd);

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
  ub4 pos,n;

  pos = mysnprintf(buf,0,len,"BT /F1 18 Tf 25 25 Td (title Tj ET\n");

  pos += mysnprintf(buf,pos,len,"BT /F1 20 Tf\n");

  // temporary: port as number
  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    y = lat2pdf(pp->lat);
    x = lon2pdf(pp->lon);
    n = mysnprintf(buf,pos,len,"1 0 0 1 %u %u Tm (%u) Tj ", x,y,port);
    if (n == 0) break;
    pos += n;
  }
  pos += mysnprintf(buf,pos,len,"ET\n");

  pos += mysnprintf(buf,pos,len,"3 w\n");

  // draw direct connection as straight line
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    dep = hp->dep;
    arr = hp->arr;
    pdep = ports + dep;
    parr = ports + arr;
    y0 = lat2pdf(pdep->lat);
    x0 = lon2pdf(pdep->lon);
    y1 = lat2pdf(parr->lat);
    x1 = lon2pdf(parr->lon);
    n = mysnprintf(buf,pos,len,"%u %u m %u %u l s\n",x0,y0,x1,y1);
    if (n == 0) break;
    pos += n;
  }
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

  fd = oscreate("net.pdf");

  pos = mysnprintf(pagebuf,0,plen,"%%PDF-1.4\n");

  // patterned after simple examples in PDF reference
  xref[pdfcat] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Catalog /Pages %u 0 R >>\nendobj\n",pdfcat,pdftree);

  xref[pdftree] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Pages /Kids [%u 0 R] /Count 1 >>\nendobj\n", pdftree,pdfnode);

  xref[pdfnode] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Page /Parent %u 0 R "
    "/MediaBox [ 0 0 %u %u ] /Contents %u 0 R /Resources"
    " << /Font << /F1 << /Type /Font /Subtype /Type1 /BaseFont /Helvetica >> >> >> >>\n"
    "endobj\n", pdfnode,pdftree,pdfscale_lon, pdfscale_lat,pdfcontent);

  xref[pdfcontent] = pos;

  xpos = pos;
  oswrite(fd,pagebuf,pos);

  cpos = addnetpdf(net,content,sizeof content);

  pos = mysnprintf(pagebuf,0, plen,"%u 0 obj\n << /Length %u >>\nstream\n", pdfcontent, cpos);
  oswrite(fd,pagebuf,pos);
  xpos += pos + cpos;

  oswrite(fd,content,cpos);

  pos = mysnprintf(pagebuf,0,plen,"endstream\nendobj\n");

  xrefpos = xpos + pos;
  pos += mysnprintf(pagebuf,pos,plen,"xref\n0 %u\n",pdflast);

  pos += mysnprintf(pagebuf,pos,plen,"%010u 65535 f \n", 0);
  for (obj = 1; obj <= pdfcontent; obj++) pos += mysnprintf(pagebuf,pos,plen,"%010u 00000 n \n", xref[obj]);

  pos += mysnprintf(pagebuf,pos,plen,"trailer\n << /Size %u /Root %u 0 R >>\n", pdflast, pdfcat);

  pos += mysnprintf(pagebuf,pos,plen,"startxref\n%u\n%s\n", xrefpos,"%%EOF");

  oswrite(fd,pagebuf,pos);
  osclose(fd);

  return 0;
}

void ininetio(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

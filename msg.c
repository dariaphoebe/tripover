// msg.c - messages

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* This file contains messaging logic used for 3 purposes:
   - console messages for the end user ( info, status, diagnostics)
   - debug logging
   - assertions

   As the program relies heavily on format flexibility, a custom print formatter
   is included here that is mostly compatible with printf, yet contains custom features

  assertions are used extensively, and are made to be enabled in production 
 */

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "base.h"

static ub4 msgfile;

#include "msg.h"

#include "os.h"
#include "util.h"
#include "time.h"

#undef hdrstop

struct filecoord {
  char name[16];
};

#define Maxmsgline 4096
#define Maxmsgfile 32

static struct filecoord filenames[Maxmsgfile];
static ub4 filendx = 1;

static enum Msglvl msglvl = Vrb;
static ub4 vrblvl = 2;
static ub4 msgopts;
int msg_doexit = 1;

static int msg_fd;
static ub4 msgwritten;

static ub4 warncnt,errcnt,assertcnt,oserrcnt;
static ub4 assertlimit;

static const char msgnames[Msglvl_last] = "XFAEWIV";
static const char *msgnames_long[Msglvl_last] = {
  "X",
  "fatal",
  "assert",
  "error",
  "warning",
  "info",
  "verbose" };

#define MSGLEN 2048
static char msgbuf[MSGLEN];
static char ccbuf[MSGLEN];
static char ccbuf2[MSGLEN];

static ub4 lastwarntoggle = 1;
static char lastwarn[MSGLEN];
static char lastwarn2[MSGLEN];
static char lasterr[MSGLEN];
static ub4 cclen,ccfln,lastwarniter,lastwarn2iter;

static char prefix[128];
static ub4 prefixlen;

static ub4 hicnts[Msglvl_last];
static ub4 hiflns[Msglvl_last];
static ub4 hicnts2[Msglvl_last];
static ub4 hiflns2[Msglvl_last];

static char himsgbufs[Msglvl_last][MSGLEN];
static char himsgbufs2[Msglvl_last][MSGLEN];

static ub4 decorpos;

static ub8 progstart;

static ub4 itercnts[Maxmsgfile * Maxmsgline];

static ub4 himsgcnts[Maxmsgfile * Maxmsgline];

// temporary choice. unbuffered is useful for console messages,
// yet debug logging may ask for buffering
static void msgwrite(const char *buf,ub4 len,int notty)
{
  int nw;

  if (len == 0) return;

  msgwritten += len;

  nw = (int)oswrite(msg_fd ? msg_fd : 1,buf,len);

  if (nw == -1) nw = (int)oswrite(2,"\nI/O error on msg write\n",24);
  if (nw == -1) oserrcnt++;
  if (msg_fd > 2 && !notty) oswrite(1,buf,len);
}

void msg_write(const char *buf,ub4 len) { msgwrite(buf,len,0); }

// make errors appear on stderr
static void myttywrite(char *buf, ub4 len)
{
  int nw;

  if (len == 0) return;

  nw = (int)oswrite(2,buf,len);
  if (nw == -1) oserrcnt++;
}

// basic %x
static ub4 xcnv(char *dst, ub4 x)
{
  ub4 n=0,c,nib = 7;

  while (nib) {
    c = (x >> (nib * 4)) & 0xf;
    if (c) break;
    nib--;
  }
  while (nib) {
    c = (x >> (nib * 4)) & 0xf;
    c += '0';
    if (c > '9') c += ('a' - '9') - 1;
    *dst++ = (char)c;
    n++;
    nib--;
  }
  c = (x & 0xf) + '0';
  if (c > '9') c += ('a' - '9') - 1;
  *dst = (char)c;

  return n+1;
}

// basic %u
static ub4 ucnv(char *dst, ub4 x,ub4 wid,char pad)
{
  ub4 n,nn;

  if (x < 10) n = 1;
  else if (x < 100) n = 2;
  else if (x < 1000) n = 3;
  else if (x < 10000) n = 4;
  else if (x < 100000) n = 5;
  else if (x < 1000000) n = 6;
  else if (x < 10000000) n = 7;
  else if (x < 100000000) n = 8;
  else if (x < 1000000000) n = 9;
  else n = 10;

  nn = n;
  if (wid > n) {
    while (wid > nn) { *dst++ = pad; nn++; }
  }
  memset(dst,' ',n);
  dst += n;
  do *--dst = (ub1)((x % 10) + '0'); while (x /= 10);
  return nn;
}

ub4 myutoa(char *dst,ub4 x) { return ucnv(dst,x,0,' '); }

// human-readable %u, 12.3G 
static ub4 Ucnv(char *dst, ub4 x1,ub4 x2,char c)
{
  ub4 n;

//  while (x2 >= 1024 - 5) { x1++; x2 >>= 10; }

  if (x1 < 10) n = 1;
  else if (x1 < 100) n = 2;
  else if (x1 < 1000) n = 3;
  else if (x1 < 10000) n = 4;
  else n = 5;

  if (x2) n += 3;

  dst += n + 1;
  *dst-- = c;
  *dst-- = ' ';

  if (x2) { // print 2 decimals
    if ((x2 % 10) > 5) x2 += 10;
    x2 /= 10;
    *dst-- = (ub1)((x2 % 10) + '0'); x2 /= 10;
    *dst-- = (ub1)((x2 % 10) + '0');
    *dst-- = '.';
  }
  do *dst-- = (ub1)((x1 % 10) + '0'); while (x1 /= 10);
  return n + 2;
}

// simple %e
static ub4 ecnv(char *dst, double x)
{
  double fexp,exp;
  ub4 xscale;
  ub4 nmant,n = 0;
  char *org = dst;
  char mantissa[32];

  if (isnan(x)) { memcpy(dst,"#NaN",4); return 4; }
  else if (isinf(x)) { memcpy(dst,"#Inf",4); return 4; }

  if (x < 0) { *dst++ = '-'; x = -x; }
  if (x < 1.0e-200) { *dst++ = '~'; *dst++ = '0'; return (ub4)(dst - org); }

  fexp = log10(x);
  if (fexp < 0) {     // |x| < 1.0
    exp = floor(fexp);
    x *= pow(10,-exp);
  } else if (fexp >= 1) { // |x| >= 10.0
    exp = floor(fexp);
    x /= pow(10,exp);
  } else {
    exp = 0;
  }
  xscale = (ub4)(x * 1.0e+6);
  memcpy(mantissa,"??????",6);
  nmant = ucnv(mantissa,xscale,0,0);
  *dst++ = mantissa[0];
  *dst++ = '.';
  if (nmant < 2) { nmant = 6; }
  memcpy(dst,mantissa + 1,nmant - 1);
  dst += nmant - 1;
  *dst++ = 'e';
  if (exp < 0) { *dst++ = '-'; exp = -exp; }
  else *dst++ = '+';
  n = ucnv(dst,(ub4)exp,0,0);

  n += (ub4)(dst - org);
  return n;
}

// simple %f
static ub4 fcnv(char *dst, ub4 len,double x)
{
  double fexp,exp;
  ub4 iexp;
  ub4 ix,n = 0,pos = 0;

  // trivia
  if (isnan(x)) { memcpy(dst,"#NaN",4); return 4; }
  else if (isinf(x)) { memcpy(dst,"#Inf",4); return 4; }
  else if (x > -1e-7 && x < 1e-7) { *dst = '0'; return 1; }
  else if (x < -4e9) { *dst++ = '-'; n = 1 + ucnv(dst,hi32,0,' '); return n; }
  else if (x > 4e9) return ucnv(dst,hi32,0,' ');

  if (x < 0) { dst[pos++] = '-'; x = -x; }

  fexp = log10(x);
  if (fexp < 0) {     // |x| < 1.0
    if (fexp < -7) { dst[pos] = '0'; return n + 1; }

    exp = floor(fexp);
    iexp = (ub4)-exp;

    dst[pos++] = '0'; dst[pos++] = '.';
    while (iexp && pos < len) { dst[pos++] = '0'; if (iexp) iexp--; }
    x *= 1e7;
    if (x > 1) {
      ix = (ub4)x;
      pos += ucnv(dst + pos,ix,0,'0');
    }
  } else { // |x| >= 1.0
    ix = (ub4)x;
    pos += ucnv(dst + pos,ix,0,'0');
    dst[pos++] = '.';
    x = (x - ix) * 1e+7;
    ix = (ub4)x;
    pos += ucnv(dst + pos,ix,7,'0');
  }
  return pos;
}

/* supports a basic subset of printf plus compatible extensions :
   %c %d %u %x %e %f %s %p %03u %-12.6s %ld %*s
   extensions are led in by '\a' preceding a conversion :
   h + %u  makes the integer formatted 'human readable' like 123.8 M for 123800000
   d + %u  minute utc to date 20140912
   u + %u  utc offset +0930   -1100
   t + %u  duration in minutes
   g + %u  distance in m or Km, passed geo units
   x + %u  %x.%u
   ` + %s  replace , with `
   v + %u%p interprets the pointer arg '%p' as an array of '%u' integers. thus :  
   int arr[] = { 23,65,23,54 };  printf("\av%u%p", 4, arr ); 
    shows  '[23 65 23 54]'
 */
static ub4 vsnprint(char *dst, ub4 pos, ub4 len, const char *fmt, va_list ap)
{
  const char *p = fmt;
  ub4 n = 0,x;
  ub4 wid,prec,plen;
  ub4 hh,mm,mdist,kmdist;
  unsigned int uval=0,vlen=0,cdval,*puval;
  unsigned long luval,lx;
  int ival,alt,padleft,do_U = 0,do_vec = 0,do_mindate = 0,do_utcofs = 0,do_xu = 0,do_dot = 0;
  int do_comma = 0,do_dist = 0,do_mindur = 0;
  long lival;
  double dval;
  char *pval;
  char c,c1,c2;
  char pad = ' ';

  if (!p || pos >= len || len - pos < 2) return 0;
  len -= pos; dst += pos; len--;

  while (*p && len - n > 2) {
    c1 = *p++;

    // extensions
    if (c1 == '\a') {
      switch(*p++) {
        case 'h': do_U = 1; break;
        case 'v': do_vec = 1; break;
        case 'V': do_vec = 2; break;
        case '.': do_dot = 1; break;
        case 'g': do_dist = 1; break;
        case 't': do_mindur = 1; break;
        case 'u': do_utcofs = 1; break;
        case 'd': do_mindate = 1; break;
        case 'D': do_mindate = 2; break;
        case 'x': do_xu = 1; break;
        case '`': do_comma = 1; break;
        case 's': if (uval != 1) dst[n++] = 's'; break;
        case '%': dst[n++] = c1; break;
        default: dst[n++] = c1;
      }
      c1 = *p; if (c1) p++;
    }
    if (c1 == '%') {
      wid = 0; pad = ' '; padleft = 0;
      prec = len;
      c2 = *p++;
      if (c2 == '-') { padleft = 1; c2 = *p++; }
      if (c2 == '*') { wid = va_arg(ap,unsigned int); c2 = *p++; }
      if (c2 == '0') pad = c2;
      while (c2 >= '0' && c2 <= '9') {
        wid = wid * 10 + (c2 - '0');
        c2 = *p++;
      }
      wid = min(wid,len - n);
      alt = 0;
      if (c2 == '#') { alt = 1; c2 = *p++; }
      else if (c2 == '.') {
        c2 = *p++;
        if (c2 == '*') { prec = va_arg(ap,unsigned int); c2 = *p++; }
        else prec = 0;
        while (c2 >= '0' && c2 <= '9') {
          prec = prec * 10 + (c2 - '0');
          c2 = *p++;
        }
      }

      // all long formats
      if (c2 == 'l') {
        c2 = *p++;
        switch(c2) {
        case 'u': luval = va_arg(ap,unsigned long);
                  uval = (ub4)min(luval,hi32);
                  if (len - n <= 10) break;
                  if (do_U && luval >= 1024UL * 10) {
                    lx = luval;
                    if (lx == hi32) n += Ucnv(dst + n,4,0,'G');
                    else if (lx >= 1024UL * 1024UL * 1024UL) n += Ucnv(dst + n,(ub4)(lx >> 30),(ub4)(lx >> 20) & 0x3ff,'G');
                    else if (lx >= 1024UL * 1024UL) n += Ucnv(dst + n,(ub4)(lx >> 20),(ub4)(lx >> 10) & 0x3ff,'M');
                    else n += Ucnv(dst + n,(ub4)(lx >> 10),(ub4)lx & 0x3ff,'K');
                  } else if (uval == hi32) {
                    memcpy(dst + n,"hi32",4); n += 4;
                  } else if (luval > hi32) {
                    n += ucnv(dst + n,(ub4)(luval >> 32),wid,pad);
                    dst[n++] = ',';
                    n += ucnv(dst + n,(ub4)luval,wid,pad);
                  } else n += ucnv(dst + n,(ub4)luval,wid,pad);
                  break;
        case 'x':
        case 'p': luval = va_arg(ap,unsigned long);
                  uval = (ub4)min(luval,hi32);
                  if (len - n <= 10) break;
                  if (luval > hi32) n += xcnv(dst + n,(ub4)(luval >> 32));
                  n += xcnv(dst + n,(ub4)luval);
                  break;
        case 'd': lival = va_arg(ap,long);
                  if (lival == 1) uval = 1;
                  else uval = hi32;
                  if (len - n <= 20) break;
                  if (lival < 0) { dst[n++] = '-'; n += ucnv(dst + n, (ub4)-lival,wid,pad); }
                  else n += ucnv(dst + n, (ub4)lival,wid,pad);
                  break;
        default: dst[n++] = c2;
        }

      // all non-long formats, including extensions
      } else {
        switch(c2) {
        case 'u': uval = va_arg(ap,unsigned int);
                  if (len - n <= 16) break;
                  if (do_vec) { vlen = uval; break; }  // save len for vector fmt

                  else if (do_dist) { // distance in geo units to Km or m
                    do_dist = 0;
                    if (uval == hi32) { dst[n++] = '-'; break; }
                    mdist = geo2m(uval);
                    kmdist = mdist / 1000;
                    if (mdist >= 5000) {
                      n += ucnv(dst + n,kmdist,wid,pad);
                      dst[n++] = '.';
                      n += ucnv(dst + n,(mdist % 1000) / 100,1,'0');
                      dst[n++] = ' '; dst[n++] = 'K'; dst[n++] = 'm';
                    } else if (mdist >= 1000) {
                      n += ucnv(dst + n,kmdist,wid,pad);
                      dst[n++] = '.';
                      n += ucnv(dst + n,(mdist % 1000) / 10,2,'0');
                      dst[n++] = ' '; dst[n++] = 'K'; dst[n++] = 'm';
                    } else {
                      n += ucnv(dst + n,mdist,wid,pad);
                      dst[n++] = ' '; dst[n++] = 'm';
                    }
                    break;

                  // dotted 123.454
                  } else if (do_dot) {
                    do_dot = 0;
                    if (uval >= 1000 * 1000) {
                      n += ucnv(dst + n,uval / 1000000,wid,'0'); dst[n++] = '.';
                      uval %= 1000000;
                      wid = 3;
                    }
                    if (uval >= 1000) {
                      n += ucnv(dst + n,uval / 1000,wid,'0'); dst[n++] = '.';
                      uval %= 1000;
                      wid = 3;
                    }
                    n += ucnv(dst + n,uval,wid,'0');
                    break;

                  // coded decimal date
                  } else if (do_mindate) {
                    if (do_mindate == 2) {
                      n += ucnv(dst + n,uval,wid,pad);
                      dst[n++] = ' ';
                    }
                    do_mindate = 0;
                    if (uval > 1440 * 356 * 10) { // minutes
                      cdval = lmin2cd(uval);
                      n += ucnv(dst + n,cdval,4,'0');
                      uval %= 1440;
                      if (uval) {
                        dst[n++] = '.';
                        hh = uval / 60; mm = uval % 60;
                        x = hh * 100 + mm;
                        n += ucnv(dst + n,x,0,' ');
                      }
                    } else if (uval > 356 * 10) { // yyyymmdd
                      cdval = day2cd(uval);
                      n += ucnv(dst + n,cdval,wid,pad);
                    } else { // minutes
                      hh = uval / 60; mm = uval % 60;
                      x = hh * 100 + mm;
                      n += ucnv(dst + n,x,0,' ');
                    }
                    break;

                  } else if (do_mindur) {  // time duration in minutes
                    do_mindur = 0;
                    if (uval == hi32) { dst[n++] = '-'; break; }
                    hh = uval / 60; mm = uval % 60;
                    if (uval >= 60) {
                      n += ucnv(dst + n,hh,wid,pad);
                      memcpy(dst + n," hour",5); n += 5;
                      if (hh > 1) dst[n++] = 's';
                      if (mm == 0) break;
                    }
                    dst[n++] = ' ';
                    n += ucnv(dst + n,mm,wid,pad);
                    memcpy(dst + n," min ",5); n += 5;
                    break;

                  // utc (timezone) offset
                  } else if (do_utcofs) {
                    do_utcofs = 0;
                    if (uval > (14 + 12) * 60) { dst[n++] = '!'; uval = (14+12) * 60; }
                    if (uval < 12 * 60) { dst[n++] = '-'; uval = 12 * 60 - uval; }
                    else { dst[n++] = '+'; uval -= 12 * 60; }
                    n += ucnv(dst + n,uval / 60,2,'0');
                    dst[n++] = ':';
                    n += ucnv(dst + n,uval % 60,2,'0');
                    break;
                  } else if (do_xu) {
                    do_xu = 0;
                    n += xcnv(dst + n,uval);
                    if (len - n <= 10) break;
                    dst[n++] = '.';
                  }

                  // human-readable 12.3M
                  if (do_U && uval >= 1024U * 10) {
                    x = uval;
                    if (x == hi32) n += Ucnv(dst + n,4,0,'G');
                    else if (x >= 1024U * 1024 * 2) n += Ucnv(dst + n,x >> 20,(x >> 10) & 0x3ff,'M');
                    else n += Ucnv(dst + n,x >> 10,x & 0x3ff,'K');

                  // regular
                  } else if (uval == hi32) {
                    memcpy(dst + n,"hi32",4); n += 4;
                  } else n += ucnv(dst + n,uval,wid,pad);
                  break;
        case 'x': uval = va_arg(ap,unsigned int);
                  if (len - n <= 10) break;
                  if (alt) { dst[n++] = '0'; dst[n++] = 'x'; }
                  n += xcnv(dst + n,uval);
                  break;
        case 'd': ival = va_arg(ap,int);
                  if (ival == 1) uval = 1;
                  else uval = hi32;
                  if (len - n <= 10) break;
                  if (ival < 0) { dst[n++] = '-'; n += ucnv(dst + n, -ival,wid,pad); }
                  else n += ucnv(dst + n,ival,wid,pad);
                  break;
        case 'e': dval = va_arg(ap,double);
                  if (len - n <= 12) break;
                  n += ecnv(dst + n,dval);
                  break;
        case 'f': dval = va_arg(ap,double);
                  if (len - n <= 16) break;
                  n += fcnv(dst + n,len - n,dval);
                  break;
        case 'c': uval = va_arg(ap,unsigned int);
                  if (isprint(uval)) dst[n++] = (char)uval;
                  else {
                    dst[n++] = '0'; dst[n++] = 'x';
                    if (len - n <= 10) break;
                    n += xcnv(dst + n,uval);
                  }
                  break;
        case 's': pval = va_arg(ap,char *);
                  if (!pval) pval = (char *)"(nil)";
                  else if ((size_t)pval < 4096) {
                    pval = (char *)" !! integer passed to printf %s !! ";
                    errcnt++;
                  }
                  plen = 0; while (plen < prec && pval[plen]) plen++;
                  if (do_comma) {
                    while ( (c = *pval++) && n < len && prec--) dst[n++] = (c == ',' ? '`' : c);
                  } else if (padleft) {
                    while (*pval && n < len && prec--) dst[n++] = *pval++;
                    while (wid > plen) { dst[n++] = pad; wid--; }
                  } else {
                    while (wid > plen) { dst[n++] = pad; wid--; }
                    while (*pval && n < len && prec) { dst[n++] = *pval++; if (prec) prec--; }
                  }
                  break;
        case 'p': puval = va_arg(ap,unsigned int *);
                  if (len - n <= 10) break;
                  if (do_vec) {
                    n += ucnv(dst + n,vlen,0,0);
                    dst[n++] = '.'; dst[n++] = '[';
                    while (vlen) {
                      uval = *puval++;
                      if (len - n <= 10) break;
                      n += ucnv(dst + n,uval,wid,pad);
                      if (do_vec == 2) {
                        dst[n++] = '.';
                        uval = *puval++;
                        if (len - n <= 10) break;
                        n += ucnv(dst + n,uval,wid,pad);
                      }
                      if (--vlen) dst[n++] = '-';
                    }
                    dst[n++] = ']'; dst[n++] = ' ';
                    do_vec = 0;
                  } else {
                    if (len - n <= 10) break;
                    luval = (unsigned long)puval;
                    if (sizeof(char *) > 4) n += xcnv(dst + n,(unsigned int)(luval >> 32));
                    n += xcnv(dst + n,(unsigned int)(luval & hi32));
                  }
                  break;
        default: dst[n++] = c2;
        }
      }
      do_U = 0;
    } else if (c1) {
      dst[n++] = c1;
    }
  }
  dst[n] = 0;
  if (n && len - n < 10) dst[n-1] = '!';
  return n;
}

ub4 myvsnprintf(char *dst, ub4 pos, ub4 len, const char *fmt, va_list ap)
{
  return vsnprint(dst,pos,len,fmt,ap);
}

ub4 __attribute__ ((format (printf,4,5))) mysnprintf(char *dst, ub4 pos, ub4 len, const char *fmt, ...)
{
  va_list ap;
  ub4 n;

  if (pos > len) return warnfln(0,0,"snprintf: pos %u > len %u",pos,len);
  va_start(ap, fmt);
  n = vsnprint(dst,pos,len,fmt,ap);
  va_end(ap);
  return n;
}

static ub4 callstack[64];
static ub4 callpos;

void enter(ub4 fln)
{
  if (callpos + 1 < Elemcnt(callstack)) callstack[callpos++] = fln;
  else infofln(fln,0,"enter above callstack");
}

extern void leave(ub4 fln)
{
  if (callpos) callpos--;
  else infofln(fln,0,"leave below callstack");
}

static void showstack(void)
{
  ub4 pos,cpos = callpos;
  char buf[256];

  while (cpos) {
    pos = msgfln(buf,0,sizeof(buf)-1,callstack[--cpos],9);
    buf[pos++] = '\n';
    msgwrite(buf,pos,0);
  }
}

// print file coords only
ub4 msgfln(char *dst,ub4 pos,ub4 len,ub4 fln,ub4 wid)
{
  ub4 line = fln & 0xffff;
  ub4 fileno = fln >> 16;

  if (fileno < Elemcnt(filenames)) return mysnprintf(dst,pos,len, "%*s %-4u ",wid,filenames[fileno].name,line);
  return mysnprintf(dst,pos,len, "*%7x* %-4u ",fileno,line);
}

static void msginfo(ub4 fln)
{
  char buf[1024];
  char xfile[32];
  ub4 len;

  ub4 line = fln & 0xffff;
  ub4 fileno = fln >> 16;
  ub8 now_usec,dusec,dsec,d100usec;
  char *filename;

  himsgcnts[min(fileno,Maxmsgfile - 1) * Maxmsgline | min(line,Maxmsgline - 1)]++;

  now_usec = gettime_usec();
  dusec = now_usec - progstart;
  dsec = dusec / (1000 * 1000);
  d100usec = (dusec % (1000 * 1000)) / 100;

  if (fileno >= Elemcnt(filenames)) {
    fmtstring(xfile,"*%x",fileno);
    filename = xfile;
  } else filename = filenames[fileno].name;

  len = fmtstring(buf,"X  %03u.%04u  %9s %4u\n",(ub4)dsec,(ub4)d100usec,filename,line);
  setmsginfo(buf,len);
}

// main message printer. supports decorated and undecorated style
static void __attribute__ ((nonnull(5))) msg(enum Msglvl lvl, ub4 sublvl, ub4 fline, ub4 code, const char *fmt, va_list ap)
{
  ub4 opts;
  ub4 pos = 0, maxlen = MSGLEN;
  sb4 n = 0;
  ub8 now_usec,dusec,dsec,d100usec;
  char lvlnam;
  ub4 iter,iterndx,itercnt,file,line;
  static ub4 himsgcnt[Maxmsgline * Maxmsgfile];

  if (fmt == NULL) { fmt = "(nil fmt)"; lvl = Error; }
  else if ((size_t)fmt < 4096) { fmt = "(int fmt)"; lvl = Error; }

  if (code & User) {
    code &= ~User;
    opts = 0;
  } else opts = msgopts;

  file = min(fline >> 16,Maxmsgfile-1);
  line = fline & hi16;
  iterndx = min(line,Maxmsgline-1);

  iter = itercnts[file * Maxmsgline | iterndx];
  itercnt = iter & hi24;
  if (itercnt < hi24) {
    itercnt++;
    itercnts[file * Maxmsgline | iterndx] = itercnt | (lvl << 24);
  }
  if (code & Iter) {
    if (itercnt > 100) return;
    else if (itercnt == 100) pos += mysnprintf(msgbuf,pos,maxlen, "  message at line %u repeated %u times\n",iterndx,itercnt);
  }

  if (opts & Msg_stamp) {
    now_usec = gettime_usec();
    dusec = now_usec - progstart;
    dsec = dusec / (1000 * 1000);
    d100usec = (dusec % (1000 * 1000)) / 100;
  } else dsec = d100usec = 0;
  if (lvl >= Msglvl_last) {
    pos += mysnprintf(msgbuf,pos,maxlen, "\nE unknown msglvl %u\n",lvl);
    lvl = Error;
  } else if (lvl <= Warn) { // precede errors with relevant past message
    if (cclen) {
      n = mysnprintf(ccbuf2,0,maxlen, "CC           %s ",ccbuf);
      n += msgfln(ccbuf2,n,maxlen,ccfln,0);
      n += mysnprintf(ccbuf2,n,maxlen, "\n");
      msgwrite(ccbuf2,n,0);
    }
    cclen = 0;
  }

  lvlnam = msgnames[lvl];

  if (*fmt == '\n') {
    msgbuf[pos++] = '\n';
    fmt++;
  }
  if (opts & Msg_type) {
    if (lvl >= Vrb) pos += mysnprintf(msgbuf, pos, maxlen, "%c%u ",lvlnam,sublvl);
    else pos += mysnprintf(msgbuf, pos, maxlen, "%c ",lvlnam);
  } else if (lvl <= Warn) pos += mysnprintf(msgbuf, pos, maxlen, "%s ",msgnames_long[lvl]);

  if (opts & Msg_stamp) pos += mysnprintf(msgbuf, pos, maxlen, "%03u.%04u ",(ub4)dsec,(ub4)d100usec);
  if (opts & Msg_pos) {
    pos += msgfln(msgbuf,pos,maxlen,fline,9);
  }

  if (prefixlen && pos + prefixlen < maxlen) {
    memcpy(msgbuf + pos,prefix,prefixlen);
    pos += prefixlen;
  }
  if (code & Ind) {
    n = min(code & Indent, maxlen - pos);
    if (n) memset(msgbuf + pos,' ',n);
    pos += n;
  }
  if (opts && pos < maxlen) msgbuf[pos++] = ' ';
  decorpos = pos;

  pos += vsnprint(msgbuf, pos, maxlen, fmt, ap);
  pos = min(pos,maxlen-1);
  msgbuf[pos] = 0;

  ub4 cnt;
  iter = himsgcnt[file * Maxmsgline | iterndx];
  cnt = iter & hi24;
  if (cnt < hi24) {
    cnt++;
    himsgcnt[file * Maxmsgline | iterndx] = cnt | (lvl << 24);
  }
  if (cnt > hicnts[lvl]) {
    hicnts[lvl] = cnt;
    hiflns[lvl] = fline;
    memcpy(himsgbufs[lvl],msgbuf,pos+1);
  } else if (cnt > hicnts2[lvl] && fline != hiflns[lvl]) {
    hicnts2[lvl] = cnt;
    hiflns2[lvl] = fline;
    memcpy(himsgbufs2[lvl],msgbuf,pos+1);
  }

  if (lvl == Warn) {
    if (lastwarntoggle) {
      memcpy(lastwarn,msgbuf,pos);
      lastwarn[pos] = 0;
      lastwarniter = cnt;
    } else {
      memcpy(lastwarn2,msgbuf,pos);
      lastwarn2[pos] = 0;
      lastwarn2iter = cnt;
    }
    lastwarntoggle ^= 1;
  }
  else if (lvl < Warn && !(code & Exit)) { memcpy(lasterr,msgbuf,pos); lasterr[pos] = 0; }
  msgbuf[pos++] = '\n';
  msgwrite(msgbuf,pos,code & Notty);
  if ( (code & Msg_ccerr) && lvl <= Warn && msg_fd != 2) myttywrite(msgbuf,pos);
}

void vmsg(enum Msglvl lvl,ub4 fln,const char *fmt,va_list ap)
{
  msg(lvl,0,fln,0,fmt,ap);
}

// exit if configured e.g. assertions
static void ccexit(int assertion)
{
  if (globs.sigint) msg_doexit = 2;
  if (msg_doexit == 0) { msg_doexit = 1; return; }
  else if (msg_doexit > 1 || assertion == 0 || assertcnt >= assertlimit) {
    eximsg();
    exit(1);
  } else msg_doexit = 1;
}

int __attribute__ ((format (printf,4,5))) genmsgfln(ub4 fln,enum Msglvl lvl,ub4 code,const char *fmt,...)
{
  va_list ap;

  msginfo(fln);

  if (msglvl < lvl) return 0;
  va_start(ap, fmt);
  msg(lvl, 0, fln, code, fmt, ap);
  va_end(ap);
  return (lvl < Warn);
}

int __attribute__ ((format (printf,2,3))) msgprefix(int rv,const char *fmt, ...)
{
  va_list ap;

  if (fmt == NULL || *fmt == 0) { prefixlen = 0; return rv; }

  va_start(ap, fmt);
  prefixlen = vsnprint(prefix,0,sizeof(prefix),fmt,ap);
  va_end(ap);
  return rv;
}

int __attribute__ ((format (printf,3,4))) vrbfln(ub4 fln, ub4 code, const char *fmt, ...)
{
  va_list ap;
  ub4 lvl;

  msginfo(fln);

  if (msglvl < Vrb) return 0;
  lvl = code / V0;
  if (lvl > vrblvl) return 0;

  va_start(ap, fmt);
  msg(Vrb, lvl, fln, code, fmt, ap);
  va_end(ap);
  return 0;
}

int __attribute__ ((format (printf,3,4))) infofln(ub4 line, ub4 code, const char *fmt, ...)
{
  va_list ap;

  msginfo(line);
  if (msglvl < Info) return 0;
  va_start(ap, fmt);
  msg(Info, 0, line, code, fmt, ap);
  va_end(ap);
  if (code & Exit) ccexit(0);
  return 0;
}

int __attribute__ ((format (printf,4,5))) infofln2(ub4 line,ub4 code,ub4 fln2,const char *fmt,...)
{
  va_list ap;

  msginfo(line);
  if (msglvl < Info) return 0;
  va_start(ap, fmt);

  if (fln2) msg(Info,0,fln2,0,"",NULL);
  msg(Info, 0, line, code, fmt, ap);
  va_end(ap);
  if (code & Exit) ccexit(0);
  return 0;
}

int info0fln(ub4 line, ub4 code, const char *s)
{
  msginfo(line);
  if (msglvl < Info) return 0;
  infofln(line, code, "%s", s);
  return 0;
}

int __attribute__ ((format (printf,3,4))) warnfln(ub4 line, ub4 code, const char *fmt, ...)
{
  va_list ap;

  msginfo(line);
  warncnt++;
  if (msglvl < Warn) return 0;
  va_start(ap, fmt);
  msg(Warn, 0, line, code, fmt, ap);
  va_end(ap);
  if (code & Exit) ccexit(0);
  return 0;
}

int __attribute__ ((format (printf,4,5))) errorfln(ub4 fln,ub4 code,ub4 fln2,const char *fmt, ...)
{
  va_list ap;

  errcnt++;

  if (fln2) msg(Info,0,fln2,0,"",NULL);
  va_start(ap, fmt);
  msg(Error, 0, fln, code, fmt, ap);
  va_end(ap);
  if (code & Exit) ccexit(0);
  else if (code & Ret0) return 0;
  return 1;
}

int __attribute__ ((format (printf,5,6))) errorccfln(ub4 fln,int cc,ub4 code,ub4 fln2,const char *fmt, ...)
{
  va_list ap;

  if (cc == 0) return 0;

  errcnt++;

  if (fln2) msg(Info,0,fln2,0,"",NULL);
  va_start(ap, fmt);
  msg(Error, 0, fln, code, fmt, ap);
  va_end(ap);
  if (code & Exit) ccexit(0);
  else if (code & Ret0) return 0;
  return 1;
}

int __attribute__ ((format (printf,3,4))) assertfln(ub4 line, ub4 code, const char *fmt, ...)
{
  va_list ap;

  showstack();

  assertcnt++;

  cclen = 0;
  va_start(ap, fmt);
  msg(Assert, 0, line, code, fmt, ap);
  va_end(ap);
  if (code & Exit) ccexit(1);
  return 1;
}


int __attribute__ ((format (printf,3,4))) osinfofln(ub4 line,ub4 code,const char *fmt, ...)
{
  va_list ap;
  char *errstr = getoserr();
  char buf[MSGLEN];

  msginfo(line);
  if (msglvl < Info) return 0;

  va_start(ap, fmt);
  vsnprint(buf,0,sizeof(buf),fmt,ap);
  va_end(ap);
  return infofln(line,code,"%s: %s",buf,errstr);
}

int __attribute__ ((format (printf,3,4))) oswarningfln(ub4 line,ub4 code,const char *fmt, ...)
{
  va_list ap;
  char *errstr = getoserr();
  char buf[MSGLEN];

  msginfo(line);
  warncnt++;
  if (msglvl < Warn) return 0;

  va_start(ap, fmt);
  vsnprint(buf,0,sizeof(buf),fmt,ap);
  va_end(ap);
  return warnfln(line,code,"%s: %s",buf,errstr);
}

int __attribute__ ((format (printf,3,4))) oserrorfln(ub4 line,ub4 code,const char *fmt, ...)
{
  va_list ap;
  char *errstr = getoserr();
  char buf[MSGLEN];

  errcnt++;
  va_start(ap, fmt);
  vsnprint(buf,0,sizeof(buf),fmt,ap);
  va_end(ap);
  return errorfln(line,code,FLN,"%s: %s",buf,errstr);
}

ub4 limit_gt_fln(ub4 x,ub4 lim,ub4 arg,const char *sx,const char *slim,const char *sarg,ub4 fln)
{
  if (lim == 0) assertfln(fln,Exit,"zero limit %s for %s",sx,slim);
  if (x < lim - 1) return x;
  if (x == lim - 1) {
    warnfln(fln,0,"limiting %s:%u to %s:%u for %s:%u",sx,x,slim,lim,sarg,arg);
    return x;
  }
  return lim;
}

void error_cc_fln(ub4 a,ub4 b,const char *sa,const char *sb,const char *cc,ub4 line,const char *fmt,...)
{
  va_list ap;

  assertfln(line,Iter,"%s:%u %s %s:%u", sa,a,cc,sb,b);
  va_start(ap,fmt);
  msg(Info,0,line,User|Iter|Ind|decorpos,fmt,ap);
  va_end(ap);
  ccexit(0);
}

void error_ge_cc_fln(ub4 a,ub4 b,const char *sa,const char *sb,ub4 line,const char *fmt,...)
{
  va_list ap;

  if (a < b) return;

  assertfln(line,0,"%s:%u >= %s:%u", sa,a,sb,b);
  va_start(ap,fmt);
  msg(Info,0,line,User|Ind|decorpos,fmt,ap);
  va_end(ap);
  ccexit(0);
}

void error_gt_cc_fln(size_t a,size_t b,const char *sa,const char *sb,ub4 line,const char *fmt,...)
{
  va_list ap;

  if (a <= b) return;

  assertfln(line,0,"%s:%lu > %s:%lu", sa,a,sb,b);
  va_start(ap,fmt);
  msg(Info,0,line,User|Ind|decorpos,fmt,ap);
  va_end(ap);
  ccexit(0);
}

int __attribute__ ((format (printf,5,6))) progress2(struct eta *eta,ub4 fln,ub4 cur,ub4 end,const char *fmt, ...)
{
  va_list ap;
  ub8 sec = 1000 * 1000,dt,est,now = gettime_usec();
  ub4 mperc,perc;
  char buf[256];
  ub4 pos,len = sizeof(buf);

  if (globs.sigint) {
    va_start(ap,fmt);
    msg(Info,0,fln,0,fmt,ap);
    va_end(ap);
    msgprefix(0,NULL);
    infofln(fln,0,"interrupted at %u from %u",cur,end);
    return 1;
  }

  if (cur == 0) {
    eta->cur = eta->end = 0;
    eta->limit = 0;
    eta->stamp = eta->start = now;
  }
  if (now - eta->stamp < 2 * sec) return 0;
  eta->stamp = now;

  va_start(ap,fmt);
  pos = vsnprint(buf,0,len,fmt,ap);
  va_end(ap);
  mperc = (ub4)(((unsigned long)cur * 1000) / end);
  mperc = min(mperc,1000);

  dt = (now - eta->start) / sec;
  if (mperc == 0) est = 0;
  else if (mperc == 1000) est = 0;
  else {
    dt = dt * 1000 / mperc;
    est = (dt * (1000UL - mperc)) / 1000;
  }
  perc = mperc / 10;
  if (est == 0) mysnprintf(buf,pos,len," %u%%",perc);
  else if (est < 120) mysnprintf(buf,pos,len," %u%% ~%u sec",perc,(ub4)est);
  else if (est < 7200) mysnprintf(buf,pos,len," %u%% ~%u min",perc,(ub4)est / 60);
  else mysnprintf(buf,pos,len," %u%% ~%u hr",perc,(ub4)est / 3600);
  infofln(fln,0,"%s",buf);
  if (eta->limit && now > eta->limit) {
    infofln(fln,0,"timelimit of %lu sec exceeded",(eta->limit - eta->start) / (1000 * 1000));
    return 1;
  } else return 0;
}

int setmsglog(const char *dir,const char *name,bool newonly)
{
  char logname[256];
  int fd,oldfd = msg_fd;
  int c,rv = 0;
  char *oldlines;
  long nr;
  ub4 n;

  if (dir && *dir) fmtstring(logname,"%s/%s",dir,name);
  else strcopy(logname,name);

  if (newonly == 0) {
    if (oldfd) info(0,"opening %slog in %s",oldfd ? "new " : "",logname);
    for (c = 9; c; c--) osrotate(logname,(const char)((c - 1) + '0'), (const char)(c + '0'));
    osrotate(logname,0,'0');
  }

  fd = oscreate(logname);

  if (fd == -1) { rv = oserror(0,"cannot create logfile %s",logname); fd = 2; }
  else if (oldfd > 2 && msgwritten && newonly == 0) {
    if (osrewind(oldfd)) rv = oserror(0,"cannot rewind %s",globs.logname);
    n = msgwritten;
    oldlines = malloc(n);
    memset(oldlines,' ',n);
    nr = osread(oldfd,oldlines,n);
    if (nr != (long)n) rv = oserror(0,"cannot read %s: %ld",globs.logname,nr);
    oswrite(fd,oldlines,n);
    osclose(oldfd);
    free(oldlines);
  }
  msgwritten = 0;

  globs.msg_fd = msg_fd = fd;
  if (oldfd && newonly == 0) info(0,"opened new log in %s from %s",logname,globs.logname);
  strcopy(globs.logname,logname);
  return rv;
}

static int dia_fd;

// level to be set beforehand
void inimsg(char *progname, const char *logname, ub4 opts)
{
  char dianame[1024];

  fmtstring(dianame,"%s.dia",globs.progname);

  // redirect stderr to file : prevent clang/gcc sanitizer output mix with our logging
  dia_fd = oscreate(dianame);
  if (dia_fd != -1) osdup2(dia_fd,2);

  progstart = gettime_usec();

  msgopts = opts;
  msgfile = setmsgfile(__FILE__);

  setmsglog(NULL,logname,0);

  infofln(0,Notty|User,"pid\t%d\tfd\t%d", globs.pid,globs.msg_fd); //on line 1:  used by dbg script

  if (msg_fd > 2) {
    infofln(FLN,Notty,"opening log %s for %s\n", logname,progname);
  }
  iniassert();
}

static void showrep(ub4 ndx)
{
  ub4 x,cnt,file,line,fln;
  enum Msglvl lvl;

  x = itercnts[ndx];
  cnt = x & hi24;
  if (cnt < 3) return;
  lvl = x >> 24;
  file = ndx / Maxmsgline;
  line = ndx % Maxmsgline;
  fln = file << 16 | line;
  genmsgfln(fln,lvl,0,"..repeated \ah%u times",cnt);
}

static void showhi(enum Msglvl lvl,ub4 lim)
{
    if (hicnts[lvl] > lim) infofln(hiflns[lvl],User,"%s *%u",himsgbufs[lvl],hicnts[lvl]);
    if (hicnts2[lvl] > lim) infofln(hiflns2[lvl],User,"%s *%u",himsgbufs2[lvl],hicnts2[lvl]);
}

void eximsg(void)
{
  ub4 i,n,n0,n1,n2,i0,i1,i2;
  int fd;


  prefixlen = 0;

  if (errcnt == 0 && assertcnt == 0 && globs.nomsgsum == 0) {

    showhi(Warn,5);
    showhi(Info,100);
    showhi(Vrb,500);

    i0 = i1 = i2 = n0 = n1 = n2 = 0;
    for (i = 0; i < Elemcnt(itercnts); i++) {
      n = itercnts[i] & hi24;
      if (n > n0) { n0 = n; i0 = i; }
      else if (n > n1) { n1 = n; i1 = i; }
      else if (n > n2) { n2 = n; i2 = i; }
    }

    showrep(i0);
    showrep(i1);
    showrep(i2);

    if (warncnt) info(0,"%u warning\as\n%s\n%s",warncnt,lastwarn,warncnt > 1 ? lastwarn2 : "");
  }

  if (assertcnt && !(assertcnt == 1 && assertlimit <= 1) ) info(0,"%u assertion\as",assertcnt);
  if (errcnt) info(0,"%u error\as\n%s",errcnt,lasterr);
  if (oserrcnt) info(0,"%u I/O error\as",oserrcnt);

  // write overall msg counts
  char *filename;
  char buf[256];
  ub4 cnt,line,fileno;
  int mfd = filecreate("tripover.msg",0);
  if (mfd != -1) {
    for (i = 0; i < Maxmsgline * Maxmsgfile; i++)
    {
      cnt = himsgcnts[i];
      if (cnt == 0) continue;
      fileno = i / Maxmsgline;
      line = i % Maxmsgline;
      filename = filenames[fileno].name;
      n = fmtstring(buf,"%s\t%u\t%u\n",filename,line,cnt);
      oswrite(mfd,buf,n);
    }
    osclose(mfd);
  }

  fd = globs.msg_fd;
  if (fd > 2) { osclose(fd); globs.msg_fd = 0; }
}

void setmsglvl(enum Msglvl lvl, ub4 vlvl,ub4 limassert)
{
  msglvl = lvl;
  vrblvl = vlvl;
  assertlimit = limassert;
}

enum Msglvl getmsglvl(void) { return msglvl; }

// make assert and debug calls more compact
ub4 setmsgfile(const char *filename)
{
  char *ext;
  struct filecoord *fc;
  ub4 len;

  if (filendx + 1 >= Elemcnt(filenames)) return 0;
  ext = strrchr(filename,'.');
  fc = filenames + filendx;
  if (ext) len = (ub4)(ext - filename);
  else len = (ub4)strlen(filename);
  len = min(len,sizeof(fc->name) - 1);
  memcpy(fc->name,filename,len);
  return (filendx++ << 16);
}

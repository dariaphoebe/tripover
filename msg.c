// msg.c - messages

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

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
#include "time.h"

#undef hdrstop

struct filecoord {
  char name[16];
};

#define Maxmsgline 4096
#define Maxmsgfile 64

static struct filecoord filenames[Maxmsgfile];
static ub4 filendx = 1;

static enum Msglvl msglvl = Vrb;
static ub4 vrblvl = 2;
static ub4 msgopts;
int msg_doexit = 1;

static int msg_fd = 2;

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
static char himsgbuf[MSGLEN];
static char himsgbuf2[MSGLEN];
static char lastwarn[MSGLEN];
static char lasterr[MSGLEN];
static ub4 cclen,ccfln;

static char prefix[128];
static ub4 prefixlen;

static ub4 hicnt,hicnt2,hifln,hifln2;
static ub4 decorpos;

static ub8 progstart;

static ub4 itercnts[Maxmsgline];

// temporary choice. unbuffered is useful for console messages,
// yet debug logging may ask for buffering
static void msgwrite(const char *buf, ub4 len)
{
  int nw;

  if (len == 0) return;

  nw = (int)oswrite(msg_fd,buf,len);

  if (nw == -1) nw = (int)oswrite(2,"\nI/O error on msg write\n",24);
  if (nw == -1) oserrcnt++;
  if (msg_fd > 2) oswrite(1,buf,len);
}

void msg_write(const char *buf,ub4 len) { msgwrite(buf,len); }

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
  dst += n;
  do *--dst = (ub1)((x % 10) + '0'); while (x /= 10);
  return nn;
}

// human-readable %u, 12.3G 
static ub4 Ucnv(char *dst, ub4 x1,ub4 x2,char c)
{
  ub4 n;

  while (x2 >= 1024 - 5) { x1++; x2 >>= 10; }

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
  if (x < 1.0e-200) { memcpy(dst,"<1e-200",7); return 7; }

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

/* supports a basic subset of printf plus compatible extensions :
   %c %d %u %x %e %s %p %03u %-12.6s %ld %*s
   extensions are led in by '\a' preceding a conversion :
   h+%u  makes the integer formatted 'human readable' like 123.8 M for 123800000
   d+%u  minute utc to date 20140912
   u+%d  utc offset +0930   -1100
   x+%u  %x.%u
   v+%u%p interprets the pointer arg '%p' as an array of '%u' integers. thus :  
   int arr[] = { 23,65,23,54 };  printf("\av%u%p", 4, arr ); 
    shows  '[23 65 23 54]'
 */
static ub4 vsnprint(char *dst, ub4 pos, ub4 len, const char *fmt, va_list ap)
{
  const char *p = fmt;
  ub4 n = 0,x;
  ub4 wid,prec,plen;
  ub4 hh,mm;
  unsigned int uval=0,vlen=0,cdval,*puval;
  unsigned long luval,lx;
  int ival,alt,padleft,do_U = 0,do_vec = 0,do_mindate = 0,do_utcofs = 0,do_xu = 0;
  long lival;
  double dval;
  char *pval;
  char c1,c2;
  char pad;

  if (!p || pos >= len || len - pos < 2) return 0;
  len -= pos; dst += pos; len--;

  while (*p && len - n > 2) {
    c1 = *p++;
    if (c1 == '\a') {
      switch(*p++) {
        case 'h': do_U = 1; break;
        case 'v': do_vec = 1; break;
        case 'u': do_utcofs = 1; break;
        case 'd': do_mindate = 1; break;
        case 'x': do_xu = 1; break;
        case 's': if (uval != 1) dst[n++] = 's'; break;
        case '%': dst[n++] = c1; c1 = '%'; break;
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
      if (c2 == 'l') {
        c2 = *p++;
        switch(c2) {
        case 'u': luval = va_arg(ap,unsigned long);
                  uval = (ub4)min(luval,hi32);
                  if (len - n <= 10) break;
                  if (do_U && luval >= 1024UL * 10) {
                    lx = luval;
                    if (lx == hi32) n += Ucnv(dst + n,4,0,'G');
                    else if (lx >= 1024UL * 1024UL) n += Ucnv(dst + n,(ub4)(lx >> 20),(ub4)(lx >> 10) & 0x3ff,'M');
                    else n += Ucnv(dst + n,(ub4)(lx >> 10),(ub4)lx & 0x3ff,'K');
                  } else if (uval == hi32) {
                    memcpy(dst + n,"hi32",4); n += 4;
                  } else if (luval > hi32) {
                    n += ucnv(dst + n,(luval >> 32),wid,pad);
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
      } else {
        switch(c2) {
        case 'u': uval = va_arg(ap,unsigned int);
                  if (len - n <= 10) break;
                  if (do_vec) { vlen = uval; break; }
                  else if (do_mindate) {
                    do_mindate = 0;
                    if (uval >= 30000000) { // minutes
                      cdval = lmin2cd(uval);
                      n += ucnv(dst + n,cdval,wid,pad);
                      uval %= 1440;
                      if (uval) dst[n++] = '.';
                    } else if (uval > 1440) { // yyyymmdd
                      cdval = lmin2cd(uval);
                      n += ucnv(dst + n,cdval,wid,pad);
                      uval %= 1440;
                      if (uval) dst[n++] = '.';
                    }
                    hh = uval / 60; mm = uval % 60;
                    x = hh * 100 + mm;
                    n += ucnv(dst + n,x,0,' ');
                    break;
                  } else if (do_utcofs) {
                    do_utcofs = 0;
                    if (uval > (14 + 12) * 60) { dst[n++] = '!'; uval = (14+12) * 60; }
                    hh = uval / 60;
                    mm = uval % 60;
                    if (uval < 12 * 60) { dst[n++] = '-'; uval = (12 - hh) * 100 - mm; } // todo
                    else { dst[n++] = '+'; uval = (hh - 12) * 100 + mm; }
                    n += ucnv(dst + n,uval,4,0);
                    break;
                  } else if (do_xu) {
                    do_xu = 0;
                    n += xcnv(dst + n,uval);
                    if (len - n <= 10) break;
                    dst[n++] = '.';
                  }
                  if (do_U && uval >= 1024U * 10) {
                    x = uval;
                    if (x == hi32) n += Ucnv(dst + n,4,0,'G');
                    else if (x >= 1024U * 1024) n += Ucnv(dst + n,x >> 20,(x >> 10) & 0x3ff,'M');
                    else n += Ucnv(dst + n,x >> 10,x & 0x3ff,'K');
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
                  if (padleft) {
                    while (*pval && n < len && prec--) dst[n++] = *pval++;
                    while (wid > plen) { dst[n++] = pad; wid--; }
                  } else {
                    while (wid > plen) { dst[n++] = pad; wid--; }
                    while (*pval && n < len && prec--) dst[n++] = *pval++;
                  }
                  break;
        case 'p': puval = va_arg(ap,unsigned int *);
                  if (len - n <= 10) break;
                  if (do_vec) {
                    n += ucnv(dst + n,vlen,0,0);
                    dst[n++] = '.'; dst[n++] = '[';
                    while (vlen--) {
                      uval = *puval++;
                      if (len - n <= 10) break;
                      n += ucnv(dst + n,uval,wid,pad);
                      if (vlen) dst[n++] = '-';
                    }
                    dst[n++] = ']'; dst[n++] = ' ';
                    do_vec = 0;
                  } else {
                    if (len - n <= 10) break;
                    luval = (unsigned long)puval;
                    if (sizeof(puval) > 4) n += xcnv(dst + n,(unsigned int)(luval >> 32));
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

  if (pos > len) return warningfln(0,0,"snprintf: pos %u > len %u",pos,len);
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
}

extern void leave(ub4 fln)
{
  if (callpos) callpos--;
  else infofln(fln,0,"leave below callstrack");
}

static void showstack(void)
{
  ub4 pos,cpos = callpos;
  char buf[256];

  while (cpos) {
    pos = msgfln(buf,0,sizeof(buf)-1,callstack[--cpos],9);
    buf[pos++] = '\n';
    msgwrite(buf,pos);
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
  ub4 iterndx,itercnt;
  static ub4 himsgcnt[Maxmsgline * Maxmsgfile];

  if (code & User) {
    code &= ~User;
    opts = 0;
  } else opts = msgopts;

  iterndx = min(fline & hi16,Maxmsgline-1);
  itercnt = itercnts[iterndx];
  if (itercnt < hi16) itercnts[iterndx] = itercnt + 1;
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
      msgwrite(ccbuf2,n);
    }
    cclen = 0;
  }

  lvlnam = msgnames[lvl];

  if (*fmt == '\n') {
    msgbuf[pos++] = '\n';
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
//  if (lvl == Assert) pos += mysnprintf(msgbuf, pos, maxlen, "assert\n  ");

  pos += vsnprint(msgbuf, pos, maxlen, fmt, ap);
  pos = min(pos,maxlen-1);

  ub4 cnt,fileno = min(fline >> 16,Maxmsgfile-1);
  cnt = himsgcnt[fileno * Maxmsgline | iterndx] + 1;
  himsgcnt[fileno * Maxmsgline | iterndx] = cnt;
  if (cnt > hicnt) {
    hicnt = cnt;
    hifln = fline;
    memcpy(himsgbuf,msgbuf,pos);
    himsgbuf[pos] = 0;
  } else if (cnt > hicnt2) {
    hicnt2 = cnt;
    hifln2 = fline;
    memcpy(himsgbuf2,msgbuf,pos);
    himsgbuf2[pos] = 0;
  }

  if (lvl == Warn) { memcpy(lastwarn,msgbuf,pos); lastwarn[pos] = 0; }
  else if (lvl < Warn && !(code & Exit)) { memcpy(lasterr,msgbuf,pos); lasterr[pos] = 0; }
  msgbuf[pos++] = '\n';
  msgwrite(msgbuf, pos);
  if ( (opts & Msg_ccerr) && lvl <= Warn && msg_fd != 2) myttywrite(msgbuf,pos);
}

void vmsg(enum Msglvl lvl,ub4 fln,const char *fmt,va_list ap)
{
  msg(lvl,0,fln,0,fmt,ap);
}

// exit if configured e.g. assertions
static void ccexit(void)
{
  if (msg_doexit == 0) { msg_doexit = 1; return; }
  else if (msg_doexit == 2 || assertcnt >= assertlimit) {
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

void __attribute__ ((format (printf,3,4))) vrbfln(ub4 fln, ub4 code, const char *fmt, ...)
{
  va_list ap;
  ub4 lvl;

  msginfo(fln);

#if 0
  if (code & CC) {
    va_list ap1;
    code &= ~CC;
    va_start(ap1, fmt);
    cclen = vsnprint(ccbuf,0,sizeof(ccbuf),fmt,ap1);
    ccfln = fln;
    va_end(ap1);
  }
#endif
  if (msglvl < Vrb) return;
  lvl = code / V0;
  if (lvl > vrblvl) return;

  va_start(ap, fmt);
  msg(Vrb, lvl, fln, code, fmt, ap);
  va_end(ap);
}

int __attribute__ ((format (printf,3,4))) infofln(ub4 line, ub4 code, const char *fmt, ...)
{
  va_list ap;

  msginfo(line);
  if (msglvl < Info) return 0;
  va_start(ap, fmt);
  msg(Info, 0, line, code, fmt, ap);
  va_end(ap);
  return 0;
}

int info0fln(ub4 line, ub4 code, const char *s)
{
  msginfo(line);
  if (msglvl < Info) return 0;
  infofln(line, code, "%s", s);
  return 0;
}

int __attribute__ ((format (printf,3,4))) warningfln(ub4 line, ub4 code, const char *fmt, ...)
{
  va_list ap;

  msginfo(line);
  warncnt++;
  if (msglvl < Warn) return 0;
  va_start(ap, fmt);
  msg(Warn, 0, line, code, fmt, ap);
  va_end(ap);
  return 0;
}

int __attribute__ ((format (printf,3,4))) errorfln(ub4 line, ub4 code, const char *fmt, ...)
{
  va_list ap;

  errcnt++;

  va_start(ap, fmt);
  msg(Error, 0, line, code, fmt, ap);
  va_end(ap);
  if (code & Exit) ccexit();
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
  if (code & Exit) ccexit();
  return 1;
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
  return warningfln(line,code,"%s: %s",buf,errstr);
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
  return errorfln(line,code,"%s: %s",buf,errstr);
}

ub4 limit_gt_fln(ub4 x,ub4 lim,ub4 arg,const char *sx,const char *slim,const char *sarg,ub4 fln)
{
  if (lim == 0) assertfln(fln,Exit,"zero limit %s for %s",sx,slim);
  if (x < lim - 1) return x;
  if (x == lim - 1) {
    warningfln(fln,0,"limiting %s:%u to %s:%u for %s:%u",sx,x,slim,lim,sarg,arg);
    return x;
  }
  return lim;
}

void error_cc_fln(ub4 a,ub4 b,const char *sa,const char *sb,const char *cc,ub4 line,const char *fmt,...)
{
  va_list ap;

  assertfln(line,0,"%s:%u %s %s:%u", sa,a,cc,sb,b);
  va_start(ap,fmt);
  msg(Info,0,line,User|Ind|decorpos,fmt,ap);
  va_end(ap);
  ccexit();
}

void error_ge_cc_fln(ub4 a,ub4 b,const char *sa,const char *sb,ub4 line,const char *fmt,...)
{
  va_list ap;

  if (a < b) return;

  assertfln(line,0,"%s:%u >= %s:%u", sa,a,sb,b);
  va_start(ap,fmt);
  msg(Info,0,line,User|Ind|decorpos,fmt,ap);
  va_end(ap);
  ccexit();
}

void error_gt_cc_fln(size_t a,size_t b,const char *sa,const char *sb,ub4 line,const char *fmt,...)
{
  va_list ap;

  if (a <= b) return;

  assertfln(line,0,"%s:%lu > %s:%lu", sa,a,sb,b);
  va_start(ap,fmt);
  msg(Info,0,line,User|Ind|decorpos,fmt,ap);
  va_end(ap);
  ccexit();
}

void __attribute__ ((format (printf,5,6))) progress2(struct eta *eta,ub4 fln,ub4 cur,ub4 end,const char *fmt, ...)
{
  va_list ap;
  ub8 sec = 1000 * 1000,dt,est,now = gettime_usec();
  ub4 perc;
  char buf[256];
  ub4 pos,len = sizeof(buf);

  vrbfln(fln,CC,"progress %u of %u",cur,end);

  va_start(ap,fmt);

  if (cur == 0) {
    eta->cur = eta->end = 0;
    eta->stamp = eta->start = now;
  } else if (cur >= end || now - eta->stamp < 2 * sec) return;
  eta->stamp = now;

  pos = vsnprint(buf,0,len,fmt,ap);
  va_end(ap);
  perc = (ub4)(((unsigned long)cur * 100) / end);
  perc = min(perc,100);

  dt = (now - eta->start) / sec;
  if (perc == 0) est = 0;
  else if (perc == 100) est = 0;
  else {
    dt = dt * 100 / perc;
    est = (dt * (100UL - perc)) / 100;
  }
  if (cur) mysnprintf(buf,pos,len," %u%%  est %u sec",perc,(ub4)est);
  infofln(fln,0,"%s",buf);
}

// level to be set beforehand
void inimsg(char *progname, const char *logname, ub4 opts)
{
  int c;
  vrb(2,"init msg for %s",logname);

  for (c = 9; c; c--) osrotate(logname,(const char)((c - 1) + '0'), (const char)(c + '0'));
  osrotate(logname,0,'0');
  msg_fd = oscreate(logname);

  if (msg_fd == -1) msg_fd = 2;
  globs.msg_fd = msg_fd;

  progstart = gettime_usec();

  msgopts = opts;
  msgfile = setmsgfile(__FILE__);
  infofln(0,User,"pid\t%d", globs.pid);

  if (msg_fd > 2 && (opts & Msg_init)) {
    infofln(0,User,"opening log %s for %s\n", logname,progname);
  }
  iniassert();
}

void eximsg(void)
{
  ub4 i,n,n0,n1,n2,i0,i1,i2;

  prefixlen = 0;

  if (errcnt == 0 && assertcnt == 0) {
    if (hicnt > 100) infofln(hifln,User,"%s *%u",himsgbuf,hicnt);
    if (hicnt2 > 100) infofln(hifln2,User,"%s *%u",himsgbuf2,hicnt2);

    i0 = i1 = i2 = n0 = n1 = n2 = 0;
    for (i = 0; i < Maxmsgline; i++) {
      n = itercnts[i];
      if (n > n0) { n0 = n; i0 = i; }
      else if (n > n1) { n1 = n; i1 = i; }
      else if (n > n2) { n2 = n; i2 = i; }
    }

    if (n0 > 1) info(0,"msg at line %u repeated %u times",i0,n0);
    if (n1 > 1) info(0,"msg at line %u repeated %u times",i1,n1);
    if (n2 > 1) info(0,"msg at line %u repeated %u times",i2,n2);

    if (warncnt) info(0,"%u warning\as\n%s",warncnt,lastwarn);
  }
  if (assertcnt && !(assertcnt == 1 && assertlimit <= 1) ) info(0,"%u assertion\as",assertcnt);
  if (errcnt) info(0,"%u error\as\n%s",errcnt,lasterr);
  if (oserrcnt) info(0,"%u I/O error\as",oserrcnt);
}

void setmsglvl(enum Msglvl lvl, ub4 vlvl,ub4 limassert)
{
  msglvl = lvl;
  vrblvl = vlvl;
  assertlimit = limassert;
}

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

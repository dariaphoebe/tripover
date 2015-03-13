// base.c - generic base utility functions

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <string.h>
#include <stdlib.h>

#include "base.h"

static ub4 msgfile;
#include "msg.h"

ub4 str2ub4(const char *s, ub4 *pv)
{
  unsigned long n;

  char *ep;

  *pv = 0;
  if (!s || !*s) return 0;
  n = strtoul(s,&ep,10);
  if (ep == s) return 0;
  if (n > hi32) *pv = hi32;
  else *pv = (ub4)n;
  return (ub4)(ep - s);
}

int hex2ub4(const char *s, ub4 *pv)
{
  unsigned long n;
  char *ep;

  *pv = 0;
  if (!s || !*s) return 1;
  n = strtoul(s,&ep,16);
  if (ep == s) return 1;
  if (n > hi32) *pv = hi32;
  else *pv = (ub4)n;
  return 0;
}

int hex2ub8(const char *s, ub8 *pv)
{
  unsigned long n;
  char *ep;

  *pv = 0;
  if (!s || !*s) return 1;
  n = strtoul(s,&ep,16);
  if (ep == s) return 1;
  *pv = n;
  return 0;
}

int str2dbl(const char *s,ub4 len,double *pv)
{
  char buf[256];
  char *endp;
  double v;

  len = min(len,sizeof(buf)-1);
  memcpy(buf,s,len);
  buf[len] = 0;
//  info(0,"len %u %s",len,buf);
  v = strtod(buf,&endp);
  *pv = 0;
  if (endp == buf) return 1;
  *pv = v;
  return 0;
}

void memcopyfln(void *dst,const void *src,size_t len,ub4 fln)
{
  char *d = dst;
  const char *s = src;

  if (len == 0) infofln(fln,0,"zero copy");
  else if (s < d && s + len > d) errorfln(fln,Exit,FLN,"overlapping copy: %p %p %lu",src,dst,(unsigned long)len);
  else if (s > d && d + len > s) errorfln(fln,Exit,FLN,"overlapping copy: %p %p %lu",src,dst,(unsigned long)len);
  else memcpy(dst,src,len);
}

void do_clear(void *p,size_t len) { memset(p,0,len); }

int strcompfln(const char *a,const char *b,const char *sa,const char *sb,ub4 fln)
{
  vrbfln(fln,0,"cmp %s %s",sa,sb);
  if (a == NULL) return errorfln(fln,Exit,FLN,"strcmp(%s:nil,%s",sa,sb);
  else if (b == NULL) return errorfln(fln,Exit,FLN,"strcmp(%s,%s:nil",sa,sb);
  else return strcmp(a,b);
}

int strncpyfln(char *dst,const char *src,ub4 len,const char *sdst,const char *ssrc,ub4 fln)
{
  error_ep_fln(dst,src,sdst,ssrc,fln);
  strncpy(dst,src,len);
  return 0;
}

int inibase(void)
{

  msgfile = setmsgfile(__FILE__);
  iniassert();

  return 0;
}

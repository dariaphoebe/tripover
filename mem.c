// mem.c - memory allocation wrappers and provisions

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Memory allocation:
   wrappers, pool allocators
 */

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "base.h"
#include "os.h"
#include "mem.h"

static ub4 msgfile;
#include "msg.h"

// soft limit for wrappers only
static const ub4 Maxmem_mb = 1024 * 8;

static ub4 totalMB;

void *alloc_fln(ub4 elems,ub4 elsize,const char *slen,const char *sel,ub1 fill,const char *desc,ub4 arg,ub4 fln)
{
  ub8 n8 = (ub8)elems * (ub8)elsize;
  size_t n;
  ub4 nm;
  void *p;

  vrbfln(fln,V0|CC,"alloc %s:\ah%u * %s:\ah%u for %s-%u",slen,elems,sel,elsize,desc,arg);

  // check for zero and overflow
  error_z_fln(elems,arg,"elems","",fln);
  error_z_fln(elsize,arg,"elsize","",fln);

  n = (size_t)n8;
  nm = (ub4)(n8 >> 20);
  if (n8 != n) error(Exit,"wraparound allocating %u MB for %s", nm, desc);

  if (nm >= Maxmem_mb) error(Exit,"exceeding %u MB limit by %u MB",Maxmem_mb,nm);
  if (totalMB + nm >= Maxmem_mb) error(Exit,"exceeding %u MB limit by %u MB",Maxmem_mb,nm + totalMB);

  p = malloc(n);
  if (!p) error(Exit,"cannot allocate %u MB for %s", nm, desc);
  else memset(p, fill, n);
  totalMB += nm;
  if (nm > 64) info(0,"alloc %u MB for %s, total %u", nm, desc, totalMB);
  return p;
}

void afree_fln(void *p,ub4 fln, const char *desc)
{
  vrbfln(fln,0,"free %p",p);
  if (!p) warningfln(fln,0,"free nil pointer for %s",desc);
  else free(p);
}

static block lrupool[256];

static block *lruhead = lrupool;
// static block *lrutail = lrupool;
static ub4 blockseq = 1;

void * __attribute__ ((format (printf,8,9))) mkblock_fln(
  block *blk,
  size_t elems,
  ub4 elsize,
  enum Blkopts opts,
  const char *selems,const char *selsize,
  ub4 fln,
  const char *fmt,...)
{
  va_list ap;
  char *desc;
  ub4 descpos,desclen;
  ub8 n8 = (ub8)elems * (ub8)elsize;
  size_t n;
  ub4 nm;
  void *p;

  error_zp(blk,fln);

  desc = blk->desc;
  desclen = sizeof(blk->desc);
  descpos = 0;

  if (fmt && *fmt) {
    va_start(ap,fmt);
    descpos = myvsnprintf(desc,0,desclen,fmt,ap);
    va_end(ap);
  }
  descpos += mysnprintf(desc,descpos,desclen," - \ah%lu %s of %u %s alloc ",(unsigned long)elems,selems,elsize,selsize);
  descpos += msgfln(desc,descpos,desclen,fln,0);
  blk->desclen = descpos;
  blk->seq = blockseq++;

  vrbfln(fln,V0|CC,"block '%s' ",desc);

  // check for zero and overflow
  error_zz(elems,elsize);

  n = (size_t)n8;
  nm = (ub4)(n8 >> 20);
  if (n8 != n) error(Exit,"wraparound allocating %u MB",nm);

  if (nm >= Maxmem_mb) error(Exit,"exceeding %u MB limit by %u MB",Maxmem_mb,nm);
  if (totalMB + nm >= Maxmem_mb) error(Exit,"exceeding %u MB limit by %u MB",Maxmem_mb,nm + totalMB);

  p = malloc(n);
  if (!p) error(Exit,"cannot allocate %u MB",nm);

  blk->base = p;
  blk->elems = elems;
  blk->elsize = elsize;
  blk->selems = selems;
  blk->selsize = selsize;
  blk->fln = fln;

  if (opts & Init0) memset(p, 0, n);
  else if (opts & Init1) memset(p, 0xff, n);

  totalMB += nm;
  if (nm > 64) info(0,"alloc %u MB for %s, total %u", nm, desc, totalMB);

  if (lruhead >= lrupool + Elemcnt(lrupool)) lruhead = lrupool;
  memcpy(lruhead,blk,sizeof(block));

  return p;
}

size_t nearblock(size_t adr)
{
  block *b = lrupool,*blklo,*blkhi;
  size_t base,top,nearlo,nearhi;
  ub4 pos;
  char buf[1024];

  nearlo = nearhi = hi32;
  blklo = blkhi = b;

  for (b = lrupool; b < lrupool + Elemcnt(lrupool); b++) {
   if (b->seq == 0) continue;
    base = (size_t)b->base;
    top = base + b->elems * b->elsize;
    if (adr < base) {
      if (base - adr < nearlo) { nearlo = base - adr; blklo = b; }
    } else if (adr >= top - 16) {
      if (adr - top < nearhi) { nearhi = adr - top; blkhi = b; }
    } else {
      return base;
    }
  }
  if (nearhi < 4096) {
    pos = mysnprintf(buf,0,sizeof buf,"%lx %u above block '%s'\n", (unsigned long)adr,(ub4)nearhi,blkhi->desc);
    oswrite(1,buf,pos);
    return (size_t)(blkhi->base);
  }
  if (nearlo < 4096) {
    pos = mysnprintf(buf,0,sizeof buf,"%lx %u below block '%s'\n", (unsigned long)adr,(ub4)nearlo,blklo->desc);
    oswrite(1,buf,pos);
    return (size_t)(blklo->base);
  }
  return adr;
}

void bound_fln(block *blk,size_t pos,ub4 elsize,const char *spos,const char *selsize,ub4 fln)
{
//  vrbfln(fln,V0|CC,"bounds on block '%s' use ",blk->desc);

  if (elsize != blk->elsize) errorfln(fln,Exit,"%s size %u on %s size %u block '%s'",selsize,elsize,blk->selsize,blk->elsize,blk->desc);
  if (pos >= blk->elems) errorfln(fln,Exit,"%s pos %lu %u above %s len %lu block '%s'",spos,pos,(ub4)(pos - blk->elems),blk->selems,blk->elems,blk->desc);
}

void inimem(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

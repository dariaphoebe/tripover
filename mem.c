// mem.c - memory allocation wrappers and provisions

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014-2015 Joris van der Geer.

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

// soft limit for wrappers only, from config
static ub4 Maxmem_mb;

static const ub4 mmap_from_mb = 32;

#define Ablocks 8192

static ub4 totalMB;

struct sumbyuse {
  char id[16];
  ub4 idlen;
  ub4 sum;
  ub4 hi,hicnt;
  char hidesc[64];
  ub4 fln,hifln;
};

struct ainfo {
  char *base;
  size_t len;
  ub4 mb;
  ub4 allocfln;
  ub4 freefln;
  ub4 alloced;
  ub4 mmap;
};

static struct sumbyuse usesums[32];

static block lrupool[256];

static block *lruhead = lrupool;

static struct ainfo ainfos[Ablocks];
static ub4 curainfo;

static const ub4 blkmagic = 0x3751455d;

static void addsum(ub4 fln,const char *desc,ub4 mbcnt)
{
  ub4 idlen = 0;
  struct sumbyuse *up = usesums;

  totalMB += mbcnt;

  while (desc[idlen] && desc[idlen] != ' ' && idlen < sizeof(up->id)-1) idlen++;
  if (idlen == 0) return;

  while (up < usesums + Elemcnt(usesums) && up->idlen && (up->idlen != idlen || memcmp(up->id,desc,idlen))) up++;
  if (up == usesums + Elemcnt(usesums)) return;
  up->idlen = idlen;
  memcpy(up->id,desc,idlen);
  up->fln = fln;
  up->sum += mbcnt;
  if (mbcnt > 512) infofln2(up->fln,0,FLN,"category %s memuse %u MB adding %u for %s",up->id,up->sum,mbcnt,desc);
  if (mbcnt > up->hi) {
    up->hi = mbcnt;
    if (up->hifln == fln) up->hicnt++;
    else up->hicnt = 1;
    up->hifln = fln;
    strcopy(up->hidesc,desc);
  }
}

static void subsum(const char *desc,ub4 mbcnt)
{
  ub4 idlen = 0;
  struct sumbyuse *up = usesums;

  totalMB -= min(mbcnt,totalMB);

  while (desc[idlen] && desc[idlen] != ' ' && idlen < sizeof(up->id)-1) idlen++;
  if (idlen == 0) return;

  while (up < usesums + Elemcnt(usesums) && up->idlen && (up->idlen != idlen || memcmp(up->id,desc,idlen))) up++;
  if (up == usesums + Elemcnt(usesums)) return;
  up->sum -= min(mbcnt,up->sum);
}

static int showedmemsums;

void showmemsums(void)
{
  struct sumbyuse *up = usesums;

  infocc(totalMB > 16,0,"total memuse %u MB",totalMB);
  while (up < usesums + Elemcnt(usesums) && up->idlen) {
    if (up->sum > 16) {
      infofln(up->fln,0,"category %s memuse %u MB",up->id,up->sum);
      infofln(up->hifln,0,"  hi %u MB * %u for %s",up->hi,up->hicnt,up->hidesc);
    }
    up++;
  }
  showedmemsums = 1;
}

void *alloc_fln(ub4 elems,ub4 elsize,const char *slen,const char *sel,ub1 fill,const char *desc,ub4 arg,ub4 fln)
{
  ub8 n8 = (ub8)elems * (ub8)elsize;
  size_t n;
  ub4 nm;
  ub4 andx;
  void *p;
  struct ainfo *ai = ainfos + curainfo;

  if (curainfo + 1 == Ablocks) errorfln(fln,Exit,FLN,"exceeding limit of %u memblocks allocating %s",Ablocks,desc);

  if (Maxmem_mb == 0) {
    vrb0(0,"setting soft VM limit to %u GB",globs.maxvm);
    Maxmem_mb = (globs.maxvm == hi24 ? hi32 : globs.maxvm * 1024);
  }
  vrbfln(fln,V0|CC,"alloc %s:\ah%u * %s:\ah%u for %s-%u",slen,elems,sel,elsize,desc,arg);

  // check for zero and overflow
  if (elems == 0) {
    infofln(fln,0,"zero length block '%s - %u'",desc,arg);
    return NULL;
  }
  error_z_fln(elsize,arg,"elsize","",fln);

  error_zp(desc,0);

  if (fill != 0 && fill != 0xff) warnfln(fln,0,"fill with %u",fill);

  n = (size_t)n8;
  nm = (ub4)(n8 >> 20);
  if (n8 != n) error(Exit,"wraparound allocating %u MB for %s", nm, desc);

  if (Maxmem_mb && nm >= Maxmem_mb) {
    showmemsums();
    errorfln(fln,Exit,FLN,"exceeding %u MB limit by %u MB for %s-%u",Maxmem_mb,nm,desc,arg);
  }
  if (Maxmem_mb && totalMB + nm >= Maxmem_mb) {
    showmemsums();
    errorfln(fln,Exit,FLN,"exceeding %u MB limit by %u+%u=%u MB for %s-%u",Maxmem_mb,totalMB,nm,nm + totalMB,desc,arg);
  }

  if (nm >= mmap_from_mb) {
    if (nm > 64) infofln2(fln,0,FLN,"alloc %u MB. for %s-%u",nm,desc,arg);
    p = osmmap(n);
    if (!p) { oserrorfln(fln,Exit,"%u: cannot allocate %u MB for %s-%u", __LINE__,nm, desc,arg); return NULL; }
    if (fill) memset(p, fill, n);
  } else {
    if (nm > 64) infofln2(fln,0,FLN,"alloc %u MB for %s-%u",nm,desc,arg);
    p = malloc(n);
    if (!p) { errorfln(fln,Exit,FLN,"cannot allocate %u MB for %s-%u", nm, desc,arg); return NULL; }
    if (nm > 128) infofln(fln,0,"clear %u MB for %s-%u",nm,desc,arg);
    memset(p, fill, n);
  }

  addsum(fln,desc,nm);

  for (andx = 0; andx < curainfo; andx++) {
    ai = ainfos + andx;
    if (p == ai->base) break;
  }
  if (curainfo && andx < curainfo) {
    if (ai->alloced) {
      doexit errorfln(fln,Exit,ai->allocfln,"previously allocated %s",desc);
    }
  } else ai = ainfos + curainfo++;

  ai->base = p;
  ai->allocfln = fln;
  ai->freefln = 0;
  ai->alloced = 1;
  ai->mmap = (nm >= mmap_from_mb);
  ai->mb = nm;
  ai->len = n;

  if (nm > 256) infofln2(fln,0,FLN,"alloced %u MB for %s, total %u", nm, desc, totalMB);
  return p;
}

int afree_fln(void *p,ub4 fln, const char *desc)
{
  block *b = lrupool;
  struct ainfo *ai = ainfos;
  ub4 andx;
  ub4 nm;

  vrbfln(fln,0,"free %p",p);
  if (!p) return warnfln(fln,0,"free nil pointer for %s",desc);

  // check if allocated with mkblock
  for (b = lrupool; b < lrupool + Elemcnt(lrupool); b++) {
    if (b->seq == 0) continue;
    if (p == b->base) return errorfln(fln,Exit,FLN,"free pointer '%s' allocated with '%s'",desc,b->desc);
  }

  // check if previously allocated
  for (andx = 0; andx < curainfo; andx++) {
    ai = ainfos + andx;
    if (p == ai->base) break;
  }
  if (andx == curainfo) return errorfln(fln,0,FLN,"free pointer %p '%s' was not allocated with alloc",p,desc);
  if (ai->freefln) {
    errorfln(fln,0,FLN,"double free of pointer %p '%s'",p,desc);
    return errorfln(ai->freefln,0,ai->allocfln,"allocated and previously freed %s",desc);
  }
  if (ai->mb > 64) infofln(ai->allocfln,0,"free %u mb from %s",ai->mb,desc);
  ai->freefln = fln;
  ai->alloced = 0;
  nm = ai->mb;
  subsum(desc,nm);
  if (ai->mmap) {
    if (osmunmap(p,ai->len)) return oserror(0,"cannot free %u MB at %p",nm,p);
  }
  else free(p);

  return 0;
}

// static block *lrutail = lrupool;
static ub4 blockseq = 1;

void * trimblock_fln(block *blk,size_t elems,ub4 elsize,const char *selems,const char *selsize,ub4 fln)
{
  void *p;
  char *desc;

  error_zp(blk,fln);

  if (blkmagic != blk->magic) errorfln(fln,Exit,FLN,"uninitialised block for trim %s:\ah%lu elems",selems,elems);
  desc = blk->desc;

  // check for zero and overflow
  if (elems == 0) errorfln(fln,Exit,FLN,"zero length block %s",desc);
  if (elsize == 0) errorfln(fln,Exit,FLN,"zero element size %s",desc);

  if (elsize != blk->elsize) errorfln(fln,Exit,FLN,"size mismatch: %s size %u on %s size %u block '%s'",selsize,elsize,blk->selsize,blk->elsize,desc);
  if (elems >= blk->elems) errorfln(fln,Exit,FLN,"%s:\ah%lu above %s:\ah%lu block '%s'",selems,elems,blk->selems,blk->elems,desc);
  p = blk->base;
  if (blk->mmap) {
    info(0,"skip realloc from \ah%lu to \ah%lu for %s",blk->elems * blk->elsize,elems * elsize,desc);
    return p;
  } else {
    p = realloc(blk->base,elems * elsize);
    if (!p) errorfln(fln,Exit,FLN,"cannot reallocate to \ah%lu for %s",elems * elsize,desc);
  }

  blk->base = p;
  blk->elems = elems;
  blk->elsize = elsize;
  blk->selems = selems;
  blk->selsize = selsize;
  blk->fln = fln;
  return p;
}

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

  if (Maxmem_mb == 0) Maxmem_mb = globs.maxvm * 1024;

  desc = blk->desc;
  desclen = sizeof(blk->desc);
  descpos = 0;

  if (blkmagic == blk->magic) errorfln(fln,Exit,FLN,"reusing block %s",desc);

  if (fmt && *fmt) {
    va_start(ap,fmt);
    descpos = myvsnprintf(desc,0,desclen,fmt,ap);
    va_end(ap);
  } else *desc = 0;

  // check for zero and overflow
  if (elems == 0) errorfln(fln,Exit,FLN,"zero length block %s",desc);
  if (elsize == 0) errorfln(fln,Exit,FLN,"zero element size %s",desc);

  blk->magic = blkmagic;

  descpos += mysnprintf(desc,descpos,desclen," - \ah%lu %s of %u %s alloc ",(unsigned long)elems,selems,elsize,selsize);
  descpos += msgfln(desc,descpos,desclen,fln,0);
  blk->desclen = descpos;
  blk->seq = blockseq++;

  vrbfln(fln,V0|CC,"block '%s' ",desc);

  // check for overflow

  n = (size_t)n8;
  nm = (ub4)(n8 >> 20);
  if (n8 != n) errorfln(fln,Exit,FLN,"wraparound allocating %u MB for %s",nm,desc);

  if (nm >= Maxmem_mb) errorfln(fln,Exit,FLN,"exceeding %u MB limit by %u MB for %s",Maxmem_mb,nm,desc);
  if (totalMB + nm >= Maxmem_mb) errorfln(fln,Exit,FLN,"exceeding %u MB limit by %u MB for %s",Maxmem_mb,nm,desc);

  if (nm >= mmap_from_mb) {
    if (nm > 1024) infofln2(fln,0,FLN,"alloc %u MB. for %s",nm,desc);
    p = osmmap(n);
    if (!p) errorfln(fln,Exit,FLN,"cannot allocate %u MB for %s",nm,desc);
    blk->mmap = 1;
    if (opts & Init1) {
      if (nm > 1024) info(0,"preset %u MB",nm);
      memset(p, 0xff, n);
    }
  } else {
    p = malloc(n);
    if (!p) errorfln(fln,Exit,FLN,"cannot allocate %u MB for %s",nm,desc);
    blk->mmap = 0;
    if (opts & Init0) memset(p, 0, n);
    else if (opts & Init1) memset(p, 0xff, n);
  }

  blk->base = p;
  blk->elems = elems;
  blk->elsize = elsize;
  blk->selems = selems;
  blk->selsize = selsize;
  blk->fln = fln;

  if (nm > 1024) infofln(fln,0,"alloc %u MB for %s, total %u", nm, desc, totalMB);

  addsum(fln,desc,nm);

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
  if (blkmagic != blk->magic) errorfln(fln,Exit,FLN,"uninitialised block in bounds check pos '%s':%lu of %s:%u",spos,(unsigned long)pos,selsize,elsize);

  if (elsize != blk->elsize) errorfln(fln,Exit,FLN,"size mismatch: %s size %u on %s size %u block '%s'",selsize,elsize,blk->selsize,blk->elsize,blk->desc);
  if (pos >= blk->elems) errorfln(fln,Exit,FLN,"%s pos %lu %u above %s len %lu block '%s'",spos,pos,(ub4)(pos - blk->elems),blk->selems,blk->elems,blk->desc);
}

ub4 meminfo(void) { return osmeminfo(); }

void inimem(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

void eximem(void)
{
  if (showedmemsums == 0) showmemsums();
}

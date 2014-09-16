// mem.c - memory alocation wrappers and provisions

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

#include "base.h"
#include "mem.h"

static ub4 msgfile;
#include "msg.h"

static const ub8 Maxmem = 1024UL * 1024 * 1024 * 4;

static ub8 totalalloc;

void *alloc_fln(ub4 elems,ub4 elsize,const char *slen,const char *sel,ub1 fill,const char *desc,ub4 arg,ub4 fln)
{
  ub8 n8 = (ub8)elems * (ub8)elsize;
  size_t n;
  ub4 nm;
  void *p;

  vrbfln(fln,V0|CC,"alloc %s:\ah%u * %s:\ah%u for %s-%u",slen,elems,sel,elsize,desc,arg);

  // check for zero and overflow
  error_zz(elems,elsize);
  error_gt2(elems,elsize,Maxmem);
  n = (size_t)n8;
  nm = (ub4)(n8 >> 20);
  if (n8 != n) error(Exit,"wraparound allocating %u MB for %s", nm, desc);
  p = malloc(n);
  if (!p) error(Exit,"cannot allocate %u MB for %s", nm, desc);
  else memset(p, fill, n);
  totalalloc += n;
  if (n > 1 * 1024 * 1024) info(0,"alloc %u MB for %s, total \ah%lu", nm, desc, totalalloc);
  return p;
}

void inimem(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

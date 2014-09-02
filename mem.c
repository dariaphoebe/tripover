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

static const ub8 Maxmem = 1024UL * 1024 * 1024 * 2;

static ub8 totalalloc;

void *alloc_fln(ub4 elems,ub4 elsize,const char *slen,const char *sel,ub1 fill,char *desc,ub4 arg,ub4 fln)
{
  ub8 n8 = elems * elsize;
  ub4 n;
  void *p;

  vrbfln(fln,V0|CC,"alloc %s:\ah%u * %s:\ah%u for %s-%u",slen,elems,sel,elsize,desc,arg);

  // check for zero and overflow
  error_zz(elems,elsize);
  error_gt2(elems,elsize,Maxmem);

  n = (ub4)n8;
  p = malloc(n);
  if (!p) error(Exit,"cannot allocate %u MB for %s", (n >> 20), desc);
  else memset(p, fill, n);
  totalalloc += n;
  if (n > 1 * 1024 * 1024) info(0,"alloc \ah%u for %s, total \ah%lu", n, desc, totalalloc);
  return p;
}

void inimem(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}
